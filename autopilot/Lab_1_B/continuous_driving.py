from advanced_mapping import scan_environment
from utils import mark_path_on_grid

import picar_4wd as fc
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
import argparse


CAR_POS = (settings.GRID_SIZE//2, 0)

stop_event = threading.Event()
traffic_cleared = threading.Event()
destination_reached = threading.Event()


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

    fps_avg_frame_count = 10

    # Initialize the object detection model
    base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
    detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    # Continuously capture images from the camera and run inference while goal not reached
    stopped = False
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
                    if category_name in ["stop sign"] and not stop_event.is_set() and not stopped:
                        print(category_name, " detected. Stopping car.")
                        stop_event.set()
                        time.sleep(3)
                        stopped = True
                        print("clearing stop event")
                        stop_event.clear()
                        traffic_cleared.set()  #todo: needs to be unset for next detection
                    # elif category_name in ["person"] and not stop_event.is_set():
                    #     print(category_name, " detected. Stopping car.")
                    #     stop_event.set()
                    #     time.sleep(3)
                    #     stop_event.clear()
                    elif stop_event.is_set():
                        # traffic cleared
                        print("Clearing stop event")
                        stop_event.clear()

        # Calculate the FPS
        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time > 1.0:  # Update the FPS every 1 second
            fps = counter / elapsed_time
            print(f"Frame rate: {fps:.2f} FPS")
            counter = 0
            start_time = time.time()

        # Print the FPS
        # print("fps: " + str(fps))

        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
            break

    cap.release()


def follow_path(path, sleep_factor=0.05, power=10):
    # Follow the path using the car
    print("following path....")
    fc.start_speed_thread()
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
                time.sleep(abs(turn_angle) * 0.02 * 0.8)
                print("Turned right by ", abs(turn_angle))
            elif turn_angle < 0:
                fc.turn_left(power)  # Adjust the power as needed
                time.sleep(abs(turn_angle) * 0.02 * 0.8)
                print("Turned left by ", abs(turn_angle))
            fc.stop()
            prev_angle = angle
            print("moving forward...")
            drive(consecutive_points)

            fc.stop()
            print("stopped. Finding next waypoint...")
            time.sleep(sleep_factor)

            i += 1

        destination_reached.set()
        print("Destination reached. Final coordinate and heading: ", path[i], prev_angle)

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
    while x < distance*0.5:
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

def destination_to_coordinates(delta_x: int, delta_y: int) -> tuple[int]:
    """
    Convert a relative destination in cm to coordinates on the grid.
    Coordinates are relative to the car's position.
    """
    # hope they don't give negative values for delta_y bc we're not moving backwards lol
    return (CAR_POS[0] + delta_x, CAR_POS[1] + abs(delta_y)) 


def calculate_straight_line_path(start, end):
    # Calculate a straight-line path from start to end
    start = np.array(start)
    end = np.array(end)

    # Calculate the direction vector
    direction = end - start

    # Calculate the distance between start and end points
    distance = np.linalg.norm(direction)

    # Normalize the direction vector
    normalized_direction = direction / distance

    # Calculate the number of points for the path (adjust as needed)
    num_points = int(distance) + 1

    # Generate the straight-line path
    straight_line_path = [tuple(np.round(start + i * normalized_direction).astype(int)) for i in range(num_points)]

    # print the straight line path
    print("*** straight line path ***: ", straight_line_path)

    return straight_line_path

def route_continuously(goal_delta_x: int, goal_delta_y: int):
    """
    Continuously route to the destination.
    When the destination is outside the range of the local grid, this function 
    takes the approach of making a path from destination to start, and subtracting
    the the distance needed until it's within the range of the local grid.
    """
    global current_grid
    global g_target
    global CAR_POS

    g_x, g_y = destination_to_coordinates(goal_delta_x, goal_delta_y)
    g_start = path_finder.Node(x=CAR_POS[0], y=CAR_POS[1])
    g_target = path_finder.Node(x=g_x, y=g_y)
    
    while True:
        # scan the environment and update the current grid
        print("scanning the environment...")
        scanned_grid = scan_environment()

        # check if the target is within the range of the current local grid
        # if it is, then run the A* algorithm and follow the path
        if (0 <= g_x <= settings.GRID_SIZE and 0 <= g_y < settings.GRID_SIZE):
            print("Condition 1: global target within range of local grid")
            # Run A* algorithm
            path = path_finder.a_star_search(scanned_grid, g_start, g_target)
            print(path)

            # Visualize the grid
            # visualize_grid(scanned_grid, path, CAR_POS, global_target)

            # Follow the path
            follow_path(path)
            print("!!!!!!!!! Path successfully followed !!!!!!!!!")
            break # stop the loop after the path is followed

        # check if the target is initially beyond the range of the current local grid
        # if it is, then move the car to the edge of the current grid and update the local grid
        elif (g_x >= settings.GRID_SIZE or g_x <= 0 or g_y >= settings.GRID_SIZE):
            print("Condition 2: global target beyond range of local grid")
            # calculate a path to the edge of the current local grid in the direction of the global target
            straight_line_path = calculate_straight_line_path(CAR_POS, (g_x, g_y))

            # find point on the straight line path that is on the edge of the local grid
            l_x, l_y = next((point for point in straight_line_path if point[0] <= 0 or point[0] >= settings.GRID_SIZE or point[1] >= settings.GRID_SIZE), None)
            l_start = path_finder.Node(x=CAR_POS[0], y=CAR_POS[1])
            l_target = path_finder.Node(x=l_x, y=l_y)
            print("local target: ", l_target)
            edge_path = path_finder.a_star_search(scanned_grid, l_start, l_target)

            # Visualize the grid
            # visualize_grid(current_grid, edge_path, CAR_POS, local_target)

            # Follow the path
            turn_angles = follow_path(edge_path)
            print("!!!!!!!!! Car has reached the edge of the local grid !!!!!!!!!")

            # CAR_POS is still at (25, 0) so adjust the frame of reference
            # the new global target is the old global target minus the distance to the edge of the local grid
            # emphasis on distance to the local grid, not just the local target since the car's position is not (0, 0)
            g_target = path_finder.Node(x=(g_x - (l_x - CAR_POS[0])), y=(g_y - (l_y - CAR_POS[1])))
            print("new global target: ", g_target)

            # update the local grid
            current_grid = np.zeros((settings.GRID_SIZE, settings.GRID_SIZE))

            # reset the car direction to 0 degrees
            print("resetting car direction to 0 degrees")
            print("turned angles: ", turn_angles)
            try:
                for angle in reversed(turn_angles):
                    if angle < 0:
                        fc.turn_right(10)  # Adjust the power as needed
                        time.sleep(abs(angle) * 0.01 * 0.8)
                    elif angle > 0:
                        fc.turn_left(10)  # Adjust the power as needed
                        time.sleep(abs(angle) * 0.01 * 0.8)
            finally:
                fc.stop()
                print("car direction reset to 0 degrees... waiting before scanning the environment again")
                time.sleep(0.5)
        else:
            print("Condition 3: global target within range of local grid but the path is not found... exiting loop")
            print("global target: ", g_target)
            break


        # wait for a few seconds before scanning the environment again
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drive a car to a specific (x, y) location.")
    parser.add_argument("x", type=int, help="X coordinate (feet), positive for right, negative for left")
    parser.add_argument("y", type=int, help="Y coordinate (feet), positive for forwards, negative for backwards")
    
    args = parser.parse_args()

    route_continuously(args.x, args.y)
