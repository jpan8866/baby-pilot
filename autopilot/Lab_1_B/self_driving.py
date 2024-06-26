from advanced_mapping import scan_environment
from utils import mark_path_on_grid, find_edge_point

import picar_4wd as fc
import vehicle_control
import threading
import path_finder
import numpy as np
import math
import settings

import sys
import time

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision


CAR_POS = (settings.GRID_SIZE//2, 0)  # start position

stop_event = threading.Event()
traffic_cleared = threading.Event()
destination_reached = threading.Event()
last_heading_in_thread = 0  # use to capture angle returned by follow_path in thread. Initialized to 0, default heading
global_car_position = CAR_POS


def run_object_detection(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
    """Continuously run inference on images acquired from the camera.

    Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
    """

    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()

    # Start capturing video input from the camera
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Initialize the object detection model
    base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
    detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    # Continuously capture images from the camera and run inference while goal not reached
    while not destination_reached.is_set():
        success, image = cap.read()
        if not success:
          sys.exit(
              'ERROR: Unable to read from webcam. Please verify your webcam settings.'
          )

        counter += 1
        image = cv2.flip(image, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = detector.detect(input_tensor)
        if detection_result:
            for detection in detection_result.detections:
                category = detection.categories[0]
                category_name = category.category_name
                probability = round(category.score, 2)
                if probability > 0.50:
                    if category_name in ["stop sign"] and not stop_event.is_set():
                        print(category_name, " detected. Stopping car.")
                        stop_event.set()
                        time.sleep(3)
                        print("clearing stop event")
                        stop_event.clear()
                        traffic_cleared.set()
                        time.sleep(3)
                    # elif category_name in ["person"] and not stop_event.is_set():
                    #     print(category_name, " detected. Stopping car.")
                    #     stop_event.set()
                    elif stop_event.is_set():
                        # traffic cleared
                        print("Clearing stop event")
                        stop_event.clear()
                        traffic_cleared.set()
        traffic_cleared.clear()

        # Calculate the FPS
        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time > 1.0:  # Update the FPS every 1 second
            fps = counter / elapsed_time
            print(f"Frame rate: {fps:.2f} FPS")
            counter = 0
            start_time = time.time()

        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
            break

    cap.release()


def follow_path(path, power=1):
    global last_heading_in_thread
    # Follow the path using the car
    print("following path....")
    i = 0
    prev_angle = 0
    angle = 0
    prev_point = (0, 0)
    try:
        while i < len(path) - 1:
            if stop_event.is_set():
                fc.stop()
                traffic_cleared.wait()
                print("Traffic cleared, resume drive")

            current_point = path[i]
            print("current point: ", current_point)

            # Calculate the angle between the current point and the next point
            if i > 0:
                angle = calculate_angle(current_point, prev_point)
                print("previous point: ", prev_point)
                print("Current heading: ", prev_angle)
                print("Next heading: ", angle)

            # Count consecutive points in the same direction
            consecutive_points = 1
            while i < len(path) - 2 and angle == calculate_angle(path[i + 1], current_point):
                i += 1
                consecutive_points += 1
            prev_point = path[i]
            print("Distance to travel (cm): ", consecutive_points)

            # Turn the car to the calculated angle if needed
            turn_angle = angle - prev_angle
            if turn_angle > 0:
                fc.turn_right(power)  # Adjust the power as needed
                time.sleep(abs(turn_angle) * 0.015 * 0.8)
                print("Turned right by ", abs(turn_angle))
            elif turn_angle < 0:
                fc.turn_left(power)  # Adjust the power as needed
                time.sleep(abs(turn_angle) * 0.015 * 0.8)
                print("Turned left by ", abs(turn_angle))
            fc.stop()
            prev_angle = angle
            print("moving forward...")
            vehicle_control.drive_calculated(consecutive_points)

            fc.stop()
            print("stopped. Finding next waypoint...")

            i += 1

        destination_reached.set()
        print("Destination reached. Final coordinate and heading: ", path[i], prev_angle)
        last_heading_in_thread = prev_angle
        return prev_angle

    finally:
        fc.stop()


def drive(distance: int, power: int = 10) -> int:
    '''
    Calculates the speed using the Photo Interruptor
    Given a distance, we drive until distance is met.
    returns traveled distance
    '''
    x = 0
    fc.forward(power)
    while x < distance * 0.85:
        if stop_event.is_set():
            fc.stop()
            traffic_cleared.wait()
            print("Traffic cleared, resume drive")
            fc.forward(power)
        time.sleep(0.05)
        s = fc.speed_val()
        x += s * 0.05
        # print("%scm" % x)
    fc.stop()
    fc.left_rear_speed.deinit()
    fc.right_rear_speed.deinit()
    return x


def calculate_angle(current_point, prev_point) -> float:
    # Calculate the angle between the current point and the next point
    delta_x = current_point[0] - prev_point[0]
    delta_y = current_point[1] - prev_point[1]
    angle_radians = math.atan2(delta_x, delta_y)
    angle_degrees = math.degrees(angle_radians)

    return angle_degrees


def route():
    scanned_grid = scan_environment()
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)

    # Define the start and goal
    start = path_finder.Node(settings.GRID_SIZE // 2, 0)
    goal = path_finder.Node(settings.GRID_SIZE // 2, settings.GRID_SIZE - 1)

    # Run A* algorithm
    path = path_finder.a_star_search(scanned_grid, start, goal)
    print("Path: ", path)
    print("Path length: ", len(path))

    # visualize map
    if path:
        mark_path_on_grid(scanned_grid, path)
    np.savetxt('./grid.txt', scanned_grid, fmt='%d')  # scp file to laptop to view

    # Visualize the grid
    # visualize_grid(scanned_grid, path, start, goal)

    path_thread = threading.Thread(target=follow_path, args=(path,))
    detection_thread = threading.Thread(target=run_object_detection, args=(settings.TF_MODEL,
                                                          settings.CAMERA_ID,
                                                          settings.FRAME_WIDTH,
                                                          settings.FRAME_HEIGHT,
                                                          settings.NUM_THREADS,
                                                          settings.ENABLE_TPU,))

    path_thread.start()
    detection_thread.start()

    path_thread.join()
    detection_thread.join()

    # # Follow the path
    # follow_path(path)


def route_continuously(dest: tuple):
    """
    goal: absolute x,y coordinates
    Continuously route to the destination.
    When the destination is outside the range of the local grid, this function
    takes the approach of making a path from destination to start, and subtracting
    the the distance needed until it's within the range of the local grid.
    """
    global last_heading_in_thread
    global global_car_position
    # if car not in direction of goal:
    #     turn car towards goal
    #     update global_car_position with the goal
    #     update goal with new frame of reference (car turned)
    if calculate_angle(dest, global_car_position) - last_heading_in_thread != 0:
        global_car_position = dest  # save global car position for next potential run
        dest = update_reference_frame(dest, global_car_position)
    else:
        # if already headed towards goal, save goal in global var for next runs
        global_car_position = dest

    # fc.start_speed_thread()
    i = 0
    start_node = path_finder.Node(settings.GRID_SIZE // 2, 0)
    while True:
        i += 1
        grid = scan_environment()
        # print(grid)

        if 0 <= dest[0] <= settings.GRID_SIZE and 0 <= dest[1] < settings.GRID_SIZE:
            print("Goal within current map")
            goal_node = path_finder.Node(dest[0], dest[1])
            path = path_finder.a_star_search(grid, start_node, goal_node)
            # print(path)
            # visualize map
            if path:
                mark_path_on_grid(grid, path)
            np.savetxt(f'./grid_{i}.txt', grid, fmt='%d')  # scp file to laptop to view
            if path is None:
                break

            path_thread = threading.Thread(target=follow_path, args=(path,))
            detection_thread = threading.Thread(target=run_object_detection, args=(settings.TF_MODEL,
                                                                                   settings.CAMERA_ID,
                                                                                   settings.FRAME_WIDTH,
                                                                                   settings.FRAME_HEIGHT,
                                                                                   settings.NUM_THREADS,
                                                                                   settings.ENABLE_TPU,))
            path_thread.start()
            detection_thread.start()

            path_thread.join()
            detection_thread.join()

            # follow_path(path)
            break
        elif (dest[0] >= settings.GRID_SIZE or dest[0] <= 0 or dest[1] >= settings.GRID_SIZE):
            print("Goal beyond current map. Go to edge and remap")
            local_goal = find_edge_point(CAR_POS, dest, settings.GRID_SIZE)

            print("local goal: ", local_goal)
            local_goal_node = path_finder.Node(local_goal[0], local_goal[1])
            local_path = path_finder.a_star_search(grid, start_node, local_goal_node)
            print(local_path)
            # visualize map
            if local_path:
                mark_path_on_grid(grid, local_path)
            np.savetxt(f'./grid_{i}.txt', grid, fmt='%d')  # scp file to laptop to view
            if local_path is None:
                print("No path generated")
                break

            path_thread = threading.Thread(target=follow_path, args=(local_path,))
            detection_thread = threading.Thread(target=run_object_detection, args=(settings.TF_MODEL,
                                                                                   settings.CAMERA_ID,
                                                                                   settings.FRAME_WIDTH,
                                                                                   settings.FRAME_HEIGHT,
                                                                                   settings.NUM_THREADS,
                                                                                   settings.ENABLE_TPU,))

            path_thread.start()
            detection_thread.start()

            path_thread.join()
            detection_thread.join()

            # heading = follow_path(local_path)
            # update global goal
            dest = (dest[0] - (local_goal[0] - CAR_POS[0]), dest[1] - (local_goal[1] - CAR_POS[1]))
            print("new goal: ", dest)

            print("updating reference frame")
            dest = update_reference_frame(dest, CAR_POS)
            print("adjusted goal to local frame of reference: ", dest)
        else:
            print("Goal point invalid: ", dest)


def update_reference_frame(dest, start) -> tuple:
    '''
    Updates the reference frame by turning the car towards goal
    the goal point is updated to fit the reference frame of the car's new heading
    '''

    # Turn car towards goal
    global last_heading_in_thread
    turn_angle = calculate_angle(dest, start) - last_heading_in_thread
    if turn_angle > 0:
        fc.turn_right(1)  # Adjust the power as needed
        time.sleep(abs(turn_angle) * 0.015 * 0.8)
        print("Turned right by ", abs(turn_angle))
    elif turn_angle < 0:
        fc.turn_left(1)  # Adjust the power as needed
        time.sleep(abs(turn_angle) * 0.015 * 0.8)
        print("Turned left by ", abs(turn_angle))
    fc.stop()

    # update goal coordinate to fit in car's new reference frame, note angle is 0 since heading toward goal
    distance = ((start[0] - dest[0])**2 + (start[1] - dest[1])**2)**0.5
    return CAR_POS[0], int(distance)


def route_continuously_no_detection(dest):
    """
    goal: absolute x,y coordinates
    Continuously route to the destination.
    When the destination is outside the range of the local grid, this function
    takes the approach of making a path from destination to start, and subtracting
    the the distance needed until it's within the range of the local grid.
    """
    global last_heading_in_thread
    global global_car_position
    # if car not in direction of goal:
    #     turn car towards goal
    #     update global_car_position with the goal
    #     update goal with new frame of reference (car turned)
    print(dest, global_car_position, last_heading_in_thread, calculate_angle(dest, global_car_position))
    if calculate_angle(dest, global_car_position) - last_heading_in_thread != 0:
        temp_dest = update_reference_frame(dest, global_car_position)
        global_car_position = dest  # save global car position for next potential run
        dest = temp_dest
    else:
        # if already headed towards goal, save goal in global var for next runs
        global_car_position = dest

    # fc.start_speed_thread()
    i = 0
    start_node = path_finder.Node(settings.GRID_SIZE // 2, 0)
    while True:
        i += 1
        grid = scan_environment()
        # print(grid)

        if 0 <= dest[0] <= settings.GRID_SIZE and 0 <= dest[1] < settings.GRID_SIZE:
            print("Goal within current map")
            goal_node = path_finder.Node(dest[0], dest[1])
            print("destination in final grid run: ", dest)
            path = path_finder.a_star_search(grid, start_node, goal_node)
            # print(path)
            # visualize map
            if path:
                mark_path_on_grid(grid, path)
            np.savetxt(f'./grid_{i}.txt', grid, fmt='%d')  # scp file to laptop to view
            if path is None:
                break

            follow_path(path)
            break
        elif (dest[0] >= settings.GRID_SIZE or dest[0] <= 0 or dest[1] >= settings.GRID_SIZE):
            print("Goal beyond current map. Go to edge and remap")
            local_goal = find_edge_point(CAR_POS, dest, settings.GRID_SIZE)

            print("local goal: ", local_goal)
            local_goal_node = path_finder.Node(local_goal[0], local_goal[1])
            local_path = path_finder.a_star_search(grid, start_node, local_goal_node)
            print(local_path)
            # visualize map
            if local_path:
                mark_path_on_grid(grid, local_path)
            np.savetxt(f'./grid_{i}.txt', grid, fmt='%d')  # scp file to laptop to view
            if local_path is None:
                print("No path generated")
                break

            last_heading_in_thread = follow_path(local_path)
            # update global goal
            dest = (dest[0] - (local_goal[0] - CAR_POS[0]), dest[1] - (local_goal[1] - CAR_POS[1]))
            print("new goal: ", dest)

            print("Point car towards goal")
            dest = update_reference_frame(dest, CAR_POS)
            print("adjusted goal to local frame of reference: ", dest)
        else:
            print("Goal point invalid: ", dest)


if __name__ == '__main__':
    # GOAL NUMBER 1
    goal = (settings.GRID_SIZE // 2, settings.GRID_SIZE*2 - 2)
    route_continuously_no_detection(goal)

    # GOAL NUMBER 2 (left turn from goal 1)
    print("MOVING TO SECOND DESTINATION")
    second_goal = (int(2.5 * settings.GRID_SIZE) - 2, settings.GRID_SIZE*2 - 2)
    route_continuously_no_detection(second_goal)