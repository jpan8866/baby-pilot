# Software based on TensorFlow examples detect.py at
#
#       https://github.com/tensorflow/examples
#
# License:
# Copyright 2024 John Pan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Main script to run the object detection routine."""
import argparse
import sys
import time

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
#import utils_apache as utils  # if you want live video feed


MODEL = 'efficientdet_lite0.tflite'
CAMERA_ID = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
NUM_THREADS = 1
ENABLE_EDGETPU = False


class ObjectDetector:
    def __init__(self, stop_event, destination_reached):
        self.stop_event = stop_event
        self.destination_reached = destination_reached

    def detect_objects(self) -> None:
        """
            Continuously run inference on images acquired from the camera.
        """

        # Variables to calculate FPS
        counter, fps = 0, 0
        start_time = time.time()

        # Start capturing video input from the camera
        cap = cv2.VideoCapture(CAMERA_ID)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        fps_avg_frame_count = 10

        # Initialize the object detection model
        base_options = core.BaseOptions(
            file_name=MODEL, use_coral=ENABLE_EDGETPU, num_threads=NUM_THREADS)
        detection_options = processor.DetectionOptions(
            max_results=3, score_threshold=0.3)
        options = vision.ObjectDetectorOptions(
            base_options=base_options, detection_options=detection_options)
        detector = vision.ObjectDetector.create_from_options(options)

        # Continuously capture images from the camera and run inference
        try:
            while not self.destination_reached.is_set() and cap.isOpened():
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
                    stopObjectInView = False
                    for detection in detection_result.detections:
                        category = detection.categories[0]
                        category_name = category.category_name
                        probability = round(category.score, 2)
                        if probability > 0.40:
                            if category_name in ["person", "stop sign"]:                        
                                print("Detected: " + category_name, "probability: " + str(probability))
                                print("Stopping...")
                                stopObjectInView = True
                                self.stop_event.set()
                    if not stopObjectInView:
                        print("Resuming...")
                        self.stop_event.clear()

                # Calculate the FPS
                # end_time = time.time()
                # elapsed_time = end_time - start_time
                # if elapsed_time > 1.0:  # Update the FPS every 1 second
                #     fps = counter / elapsed_time
                #     # print(f"Frame rate: {fps:.2f} FPS")
                #     counter = 0
                #     start_time = time.time()

                # Print the FPS
                # print("fps: " + str(fps))
        finally:
            cap.release()
