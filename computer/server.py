from flask import Flask, request, Response, render_template_string
import numpy as np
import cv2
import threading
import time

app = Flask(__name__)

# Global variable to store the latest frame and a lock for thread safety
latest_frame = None
frame_lock = threading.Lock()

<<<<<<< HEAD
# Lane detection function
# def detect_lanes(frame):
#     """Detects black tape lanes in the frame using edge detection."""
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
#     blur = cv2.GaussianBlur(gray, (5, 5), 0)  # Reduce noise
#     edges = cv2.Canny(blur, 50, 150)  # Detect edges
    
#     # Define a region of interest (ROI)
#     height, width = frame.shape[:2]
#     mask = np.zeros_like(edges)
#     roi = np.array([[
#         (50, height),  # Bottom-left
#         (width - 50, height),  # Bottom-right
#         (width // 2, height // 2)  # Top-center
#     ]], np.int32)
#     cv2.fillPoly(mask, roi, 255)
    
#     # Apply ROI mask
#     masked_edges = cv2.bitwise_and(edges, mask)
    
#     # Detect lines using Hough Transform
#     lines = cv2.HoughLinesP(masked_edges, 2, np.pi/180, 100, minLineLength=40, maxLineGap=5)
    
#     # Draw detected lanes
#     lane_image = np.zeros_like(frame)
#     if lines is not None:
#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 cv2.line(lane_image, (x1, y1), (x2, y2), (0, 0, 255), 5)
    
#     # Overlay detected lanes on the original frame
#     processed_frame = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)
#     return processed_frame

# In Flask app.py
steering_data = {"angle": 0, "speed": 0}

@app.route('/set_steering', methods=['POST'])
def set_steering():
    global steering_data
    steering_data = request.get_json()
    return "OK", 200

@app.route('/get_steering')
def get_steering():
    return jsonify(steering_data)

def detect_lanes(frame):
    """Lane detection with full diagnostic overlay"""
    height, width = frame.shape[:2]
    debug_frame = frame.copy()
    status_y = 30  # Starting Y-position for status text
    line_spacing = 30
    
    # --- Stage 1: Frame Reception Check ---
    cv2.putText(debug_frame, f"Frame received: {'Yes' if frame is not None else 'No'}",
                (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    status_y += line_spacing

    # --- Stage 2: Edge Detection ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    
    edge_pixels = cv2.countNonZero(edges)
    cv2.putText(debug_frame, f"Edges detected: {edge_pixels} pixels",
                (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                (0, 255, 0) if edge_pixels > 1000 else (0, 0, 255), 2)
    status_y += line_spacing

    # --- Stage 3: ROI Mask Application ---
    roi_vertices = np.array([[ 
        (int(width * 0.1), height),
        (int(width * 0.9), height),
        (int(width * 0.6), int(height * 0.6)),
        (int(width * 0.4), int(height * 0.6))
    ]], dtype=np.int32)
    
    # Draw ROI polygon on debug frame
    cv2.polylines(debug_frame, roi_vertices, isClosed=True, 
                 color=(255, 0, 255), thickness=2)
    cv2.putText(debug_frame, "ROI Applied", (10, status_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    status_y += line_spacing

    # --- Stage 4: Hough Line Detection ---
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, roi_vertices, 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    lines = cv2.HoughLinesP(masked_edges, 2, np.pi/180, 50, 
                            minLineLength=40, maxLineGap=20)
    
    line_count = len(lines) if lines is not None else 0
    cv2.putText(debug_frame, f"Hough lines found: {line_count}",
                (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0) if line_count > 0 else (0, 0, 255), 2)
    status_y += line_spacing

    # --- Stage 5: Lane Classification ---
    left_lines = []
    right_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1: 
                continue  # Avoid division by zero
            slope = (y2 - y1) / (x2 - x1)
            
            # Classify left/right lanes based on slope
            if slope < -0.5:  # Likely a left lane
                left_lines.append(line[0])
            elif slope > 0.5:  # Likely a right lane
                right_lines.append(line[0])

    cv2.putText(debug_frame, 
                f"Left lines: {len(left_lines)} | Right lines: {len(right_lines)}",
                (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0) if (len(left_lines)+len(right_lines)) > 0 else (0,0,255), 2)
    status_y += line_spacing

    # --- Stage 6: Lane Averaging ---
    def average_lines(lines, y_min, y_max):
        if not lines:
            return None
        # Combine all line points
        x = np.concatenate([[x1, x2] for x1, y1, x2, y2 in lines])
        y = np.concatenate([[y1, y2] for x1, y1, x2, y2 in lines])
        try:
            coeffs = np.polyfit(y, x, 1)
        except Exception as e:
            return None
        x_min = int(coeffs[0] * y_min + coeffs[1])
        x_max = int(coeffs[0] * y_max + coeffs[1])
        return [x_min, y_min, x_max, y_max]

    y_min = int(height * 0.6)
    y_max = height
    avg_left = average_lines(left_lines, y_min, y_max)
    avg_right = average_lines(right_lines, y_min, y_max)

    # Draw averaged lanes if detected
    lane_status = "No lanes"
    if avg_left and avg_right:
        cv2.line(debug_frame, (avg_left[0], avg_left[1]), 
                 (avg_left[2], avg_left[3]), (255, 0, 0), 5)
        cv2.line(debug_frame, (avg_right[0], avg_right[1]), 
                 (avg_right[2], avg_right[3]), (0, 255, 0), 5)
        lane_status = "Both lanes detected"
    elif avg_left or avg_right:
        lane_status = "Partial detection"
    
    cv2.putText(debug_frame, f"Final Status: {lane_status}",
                (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0) if lane_status.startswith("Both") else (0,0,255), 2)
    status_y += line_spacing

    # --- Stage 7: Steering Calculation & Command Dispatch ---
    steering_angle = 0  # Default: go straight
    if avg_left and avg_right:
        lane_center = (avg_left[0] + avg_right[0]) // 2
        frame_center = width // 2
        error_px = lane_center - frame_center
        steering_angle = error_px * 0.01  # Tuning multiplier

    # Send steering command to ESP32-CAM
    requests.post("http://ESP32_CAM_IP/set_steering", 
                  json={"angle": steering_angle, "speed": 50})

    # Return the debug frame with all overlays
    return debug_frame


=======
>>>>>>> parent of 45221a8 (Add Annotation)
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

