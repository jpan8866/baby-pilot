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


CAR_POS = (settings.GRID_SIZE//2, 0)

stop_event = threading.Event()
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
                        print("clearing stop event")
                        stop_event.clear()
                    elif category_name in ["person"] and not stop_event.is_set():
                        print(category_name, " detected. Stopping car.")
                        stop_event.set()
                        time.sleep(3)
                        stop_event.clear()
                    elif stop_event.is_set():
                        # traffic cleared
                        print("Clearing stop event")
                        stop_event.clear()

        # Clear the stop event if no stop sign has been detected for a while
        if stop_sign_detected and time.time() - last_stop_sign_detection_time > 3:
            print("Stop sign no longer detected. Resuming car.")
            stop_sign_detected = False
            stop_event.clear()

            def is_traffic_clear():
                pass

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
                print("Traffic detected. Stopping car.")
                fc.stop()
                stop_event.wait()
                print("Traffic cleared. Resuming drive.")

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
                time.sleep(abs(turn_angle) * 0.01 * 0.8)
                print("Turned right by ", abs(turn_angle))
            elif turn_angle < 0:
                fc.turn_left(power)  # Adjust the power as needed
                time.sleep(abs(turn_angle) * 0.01 * 0.8)
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
    while x < distance*0.95:
        if stop_event.is_set():
            fc.stop()
            stop_event.wait()
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
    path = path_finder.a_star_search_4dir(scanned_grid, start, goal)
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


if __name__ == '__main__':
    route()
