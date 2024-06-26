from flask import Flask, Response
import cv2

app = Flask(__name__)


def generate_frames():
    camera = cv2.VideoCapture(0)  # Use the appropriate camera index
    if not camera.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # give out frame using generator
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    # use multipart/x-mixed-replace to tell client that content will be replaced with new content (i.e. next frame)
    # simulates smooth video playback
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def start_video_feed():
    app.run(host="0.0.0.0", port=5000)  # make it accessible on IP address of pi