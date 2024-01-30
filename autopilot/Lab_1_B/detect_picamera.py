import time
import picamera
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import numpy as np

# Initialize the object detection model (replace with your own model initialization)
model_path = 'efficientdet_lite0.tflite'
base_options = core.BaseOptions(file_name=model_path, num_threads=4)
detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)

# Create a PiCamera object
with picamera.PiCamera() as camera:
    # Set video resolution and frame rate (adjust as needed)
    camera.resolution = (1280, 720)
    camera.framerate = 30  # Set your desired frame rate

    # Enable hardware acceleration for video encoding
    camera.video_stabilization = False
    camera.image_effect = 'none'
    camera.color_effects = None
    camera.exposure_mode = 'auto'
    camera.awb_mode = 'auto'
    camera.rotation = 0
    camera.hflip = False
    camera.vflip = False

    # Create a video stream for capturing frames
    stream = picamera.PiCameraCircularIO(camera, seconds=5)

    # Start capturing video into the circular buffer
    camera.start_recording(stream, format='h264')

    try:
        frame_count = 0
        start_time = time.time()

        while True:
            # Capture the latest frame from the circular buffer
            camera.wait_recording(0)
            frame = stream.getvalue()
            # Check if the frame is empty or invalid
            if frame is None or len(frame) == 0:
                continue  # Skip processing this frame
            # Convert the frame to an RGB image
            # from: https://stackoverflow.com/questions/65375839/read-last-frame-from-picamera-ring-buffer
            # camera.capture(rawCapture, format="bgr", use_video_port=True)
            # frame = rawCapture.array
            image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Create a TensorImage object from the RGB image.
            input_tensor = vision.TensorImage.create_from_array(rgb_image)

            # Run object detection estimation using the model.
            detection_result = detector.detect(input_tensor)

            # Process the detection result (replace this part with your own logic)
            if detection_result:
                for detection in detection_result:
                    print(f"Object: {detection.labels[0]}")
                    print(f"Probability: {detection.scores[0]}")

            frame_count += 1

            # Calculate time elapsed
            end_time = time.time()
            elapsed_time = end_time - start_time

            # Calculate the frame rate
            if elapsed_time > 1.0:  # Update the FPS every 1 second
                fps = frame_count / elapsed_time
                print(f"Frame rate: {fps:.2f} FPS")
                frame_count = 0
                start_time = time.time()

    finally:
        # Stop recording when finished
        camera.stop_recording()
