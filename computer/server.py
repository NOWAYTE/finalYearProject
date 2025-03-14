from flask import Flask, request, Response, render_template_string
import numpy as np
import cv2
import threading
import time

app = Flask(__name__)

# Global variable to store the latest frame and a lock for thread safety
latest_frame = None
frame_lock = threading.Lock()

@app.route('/upload', methods=['POST'])
def upload():
    global latest_frame
    # Convert the incoming JPEG byte stream to a numpy array
    img_array = np.frombuffer(request.data, dtype=np.uint8)
    # Decode the image using OpenCV
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    
    if frame is not None:
        with frame_lock:
            latest_frame = frame
        return "Frame received", 200
    else:
        return "Invalid frame", 400

def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                # Wait for a frame to be available
                time.sleep(0.1)
                continue
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', latest_frame)
        if not ret:
            continue
        frame = buffer.tobytes()
        # Yield the frame in the multipart MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)  # Adjust this delay to control the stream frame rate

@app.route('/video_feed')
def video_feed():
    # Stream the MJPEG frames
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    # A simple HTML page to display the MJPEG stream
    html = '''
    <!doctype html>
    <html>
      <head>
        <title>ESP32 Video Stream</title>
      </head>
      <body>
        <h1>Live Stream from ESP32 Camera</h1>
        <img src="/video_feed" width="640" height="480">
      </body>
    </html>
    '''
    return render_template_string(html)

if __name__ == '__main__':
    # Run the Flask server on all network interfaces at port 5000
    app.run(host='0.0.0.0', port=5000, debug=True)

