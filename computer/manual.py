from flask import Flask, request, Response, render_template_string
import numpy as np
import cv2
import threading
import time
from bleak import BleakClient
import queue
import asyncio

app = Flask(__name__)

# BLE Configuration
HM10_ADDRESS = "D8:B6:73:0D:B5:A9"
UUID_SERVICE = "0000FFE0-0000-1000-8000-00805F9B34FB"
UUID_CHARACTERISTIC = "0000FFE1-0000-1000-8000-00805F9B34FB"

# Global variables
latest_frame = None
frame_lock = threading.Lock()
ble_queue = queue.Queue()

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

@app.route('/upload', methods=['POST'])
def upload():
    global latest_frame
    try:
        img_array = np.frombuffer(request.data, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        if frame is not None:
            with frame_lock:
                latest_frame = frame.copy()

        return "OK", 200
    except Exception as e:
        print(f"Frame接收错误: {e}")
        return "ERROR", 500

@app.route('/control', methods=['POST'])
def control():
    command = request.form.get('command', 'S')
    ble_queue.put(command)
    return "OK", 200

def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            ret, buffer = cv2.imencode('.jpg', latest_frame)
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
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>RC Car Controller</title>
        <style>
            .container {
                display: flex;
                flex-direction: column;
                align-items: center;
            }
            .controls {
                margin: 20px;
                display: grid;
                grid-template-columns: repeat(3, 1fr);
                gap: 10px;
            }
            button {
                padding: 15px 25px;
                font-size: 18px;
                cursor: pointer;
            }
            #videoFeed {
                border: 2px solid #333;
                margin: 10px;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>RC Car Controller</h1>
            <img id="videoFeed" src="/video_feed" width="640" height="480">
            
            <div class="controls">
                <button onclick="sendCommand('F')">前进 Forward</button>
                <button onclick="sendCommand('L')">左转 Left</button>
                <button onclick="sendCommand('S')">停止 Stop</button>
                <button onclick="sendCommand('R')">右转 Right</button>
                <button onclick="sendCommand('B')">后退 Back</button>
            </div>
        </div>

        <script>
            function sendCommand(cmd) {
                fetch('/control', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/x-www-form-urlencoded',
                    },
                    body: 'command=' + cmd
                });
            }
        </script>
    </body>
    </html>
    ''')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
