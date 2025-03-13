import socketserver
import http.server
import cv2
import numpy as np
import urllib.request
from urllib.parse import urlparse, parse_qs
import threading
import time

ESP32_CAM_URL = "http://192.168.1.101:81/stream"
sensor_data = None
car_stopped = False
lock = threading.Lock()  # For thread-safe variable access

class SensorDataHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        global sensor_data, car_stopped
        if self.path.startswith("/sensor"):
            try:
                query = urlparse(self.path).query
                params = parse_qs(query)
                distance = float(params.get('distance', [0])[0])
                
                with lock:
                    sensor_data = distance
                    print(f"Distance: {sensor_data} cm")

                    if sensor_data < 20 and not car_stopped:
                        print("Stopping car...")
                        car_stopped = True
                        # Implement car stop command

                    elif sensor_data >= 20 and car_stopped:
                        print("Resuming car...")
                        car_stopped = False
                        # Implement car resume command

                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception as e:
                print(f"Error: {e}")
                self.send_response(500)
                self.end_headers()

def stream_video():
    # Improved MJPEG stream handling
    stream = urllib.request.urlopen(ESP32_CAM_URL)
    bytes = b''
    while True:
        bytes += stream.read(1024)
        a = bytes.find(b'\xff\xd8')
        b = bytes.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes[a:b+2]
            bytes = bytes[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('ESP32-CAM Stream', frame)
            if cv2.waitKey(1) == ord('q'):
                break
    cv2.destroyAllWindows()

def run_http_server():
    server = socketserver.TCPServer(("0.0.0.0", 8002), SensorDataHandler)
    print("Server started on port 8002")
    server.serve_forever()

if __name__ == "__main__":
    threading.Thread(target=run_http_server, daemon=True).start()
    threading.Thread(target=stream_video, daemon=True).start()
    # Keep main thread alive
    while True:
        time.sleep(1)
