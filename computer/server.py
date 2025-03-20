from flask import Flask, request, Response, render_template_string
import numpy as np
import cv2
import threading
import time
from ultralytics import YOLO
import asyncio
from bleak import BleakClient
import queue


app = Flask(__name__)

# BLE Configuration
HM10_ADDRESS = "D8:B6:73:0D:B5:A9"
UUID_SERVICE = "0000FFE0-0000-1000-8000-00805F9B34FB"
UUID_CHARACTERISTIC = "0000FFE1-0000-1000-8000-00805F9B34FB"

# Global variables
latest_frame = None
frame_lock = threading.Lock()
ble_queue = queue.Queue()
ble_event = threading.Event()

# Load YOLO model
yolo_model = YOLO("yolov8n.pt")  # Consider yolov8n-seg for smaller model

async def ble_consumer():
    client = None
    while True:
        try:
            if not client or not client.is_connected:
                client = BleakClient(HM10_ADDRESS)
                await client.connect()
                print("BLE Connected")
            
            command = await asyncio.to_thread(ble_queue.get, True)
            await client.write_gatt_char(
                UUID_CHARACTERISTIC, 
                command.encode() + b'\n'
            )
            print(f"Sent: {command}")
            
        except Exception as e:
            print(f"BLE Error: {e}")
            await asyncio.sleep(1)

def ble_background():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_consumer())

# Start BLE thread
ble_thread = threading.Thread(target=ble_background, daemon=True)
ble_thread.start()


def calculate_steering(frame, avg_left, avg_right, width, height):
    if avg_left and avg_right:
        left_x = avg_left[0]  # Bottom x of left lane
        right_x = avg_right[0]  # Bottom x of right lane
        lane_center = (left_x + right_x) // 2
        frame_center = width // 2
        
        # Calculate deviation from center and draw a reference line
        deviation = lane_center - frame_center
        cv2.line(frame, (frame_center, height), (lane_center, height), (0, 255, 255), 5)
        
        # Determine steering command based on deviation thresholds
        if abs(deviation) < 30:  # Dead zone to prevent jitter
            return "F"
        elif deviation > 0:
            return "F"
        else:
            return "F"
    elif avg_left:
        return "F"  # Only left lane detected, steer right
    elif avg_right:
        return "F"   # Only right lane detected, steer left
    else:
        return "F"   # Emergency stop

def detect_lanes(frame):
    """Lane detection with full diagnostic overlay"""
    height, width = frame.shape[:2]
    debug_frame = frame.copy()
    status_y = 30  # Starting Y-position for status text
    line_spacing = 30

    # --- Stage 1: Frame Reception Check ---
    cv2.putText(debug_frame, f"Frame received: Yes", (10, status_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    status_y += line_spacing

    # --- Stage 2: Edge Detection ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    edge_pixels = cv2.countNonZero(edges)
    cv2.putText(debug_frame, f"Edges detected: {edge_pixels} pixels", (10, status_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if edge_pixels > 1000 else (0, 0, 255), 2)
    status_y += line_spacing

    # --- Stage 3: ROI Mask Application ---
    roi_vertices = np.array([[
        (int(width * 0.1), height),
        (int(width * 0.9), height),
        (int(width * 0.6), int(height * 0.6)),
        (int(width * 0.4), int(height * 0.6))
    ]], dtype=np.int32)
    cv2.polylines(debug_frame, roi_vertices, isClosed=True, color=(255, 0, 255), thickness=2)
    cv2.putText(debug_frame, "ROI Applied", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    status_y += line_spacing

    # --- Stage 4: Hough Line Detection ---
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, roi_vertices, 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    lines = cv2.HoughLinesP(masked_edges, 2, np.pi/180, 50, minLineLength=40, maxLineGap=20)
    line_count = len(lines) if lines is not None else 0
    cv2.putText(debug_frame, f"Hough lines found: {line_count}", (10, status_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if line_count > 0 else (0, 0, 255), 2)
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
            if slope < -0.5:  # Left lane (negative slope)
                left_lines.append(line[0])
            elif slope > 0.5:  # Right lane (positive slope)
                right_lines.append(line[0])
    cv2.putText(debug_frame, f"Left lines: {len(left_lines)} | Right lines: {len(right_lines)}", 
                (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0) if (len(left_lines)+len(right_lines)) > 0 else (0, 0, 255), 2)
    status_y += line_spacing

    # --- Stage 6: Lane Averaging ---
    def average_lines(lines, y_min, y_max):
        if not lines:
            return None
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
        cv2.line(debug_frame, (avg_left[0], avg_left[1]), (avg_left[2], avg_left[3]), (255, 0, 0), 5)
        cv2.line(debug_frame, (avg_right[0], avg_right[1]), (avg_right[2], avg_right[3]), (0, 255, 0), 5)
        lane_status = "Both lanes detected"
    elif avg_left or avg_right:
        lane_status = "Partial detection"
    cv2.putText(debug_frame, f"Final Status: {lane_status}", (10, status_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if lane_status.startswith("Both") else (0, 0, 255), 2)
    
    # --- Calculate Steering Command ---
    command = calculate_steering(debug_frame, avg_left, avg_right, width, height)
    cv2.putText(debug_frame, f"Command: {command}", (10, status_y + 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if command == "forward" else (0, 0, 255), 2)

    # --- YOLO Object Detection ---
    results = yolo_model(frame)  # Run YOLO on the frame
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()  # Get bounding boxes
        confidences = result.boxes.conf.cpu().numpy()  # Get confidence scores
        class_ids = result.boxes.cls.cpu().numpy()  # Get class IDs

        for box, conf, cls_id in zip(boxes, confidences, class_ids):
            x1, y1, x2, y2 = map(int, box)
            label = f"{yolo_model.names[int(cls_id)]} {conf:.2f}"
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return debug_frame, command

@app.route('/upload', methods=['POST'])
def upload():
    global latest_frame
    try:
        img_array = np.frombuffer(request.data, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        
        if frame is None:
            return "S", 400

        processed_frame, command = detect_lanes(frame)
        ble_queue.put(command)  # Send to BLE queue
        
        with frame_lock:
            latest_frame = processed_frame

        return command, 200
    
    except Exception as e:
        print(f"Processing error: {e}")
        return "S", 500

def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            ret, buffer = cv2.imencode('.jpg', latest_frame)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
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
    app.run(host='0.0.0.0', port=5000, debug=True)
