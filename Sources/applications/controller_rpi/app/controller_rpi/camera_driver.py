# camera_driver.py
import sys
sys.path.append("/usr/lib/python3/dist-packages")

import io
import time
import threading
import cv2
from picamera2 import Picamera2
from threading import Lock

class CameraDriver:
    def __init__(self, width=640, height=480, fps=15):
        self.width = width
        self.height = height
        self.fps = fps
        self.picam2 = None
        self.running = False
        self.frame_lock = Lock()
        self.latest_frame = None
        self.thread = None

    def start(self):
        if self.running:
            print("[Camera] Already running.")
            return
        print("[Camera] Starting camera...")
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def stop(self):
        if not self.running:
            print("[Camera] Not running.")
            return
        print("[Camera] Stopping camera...")
        self.running = False
        if self.picam2:
            try:
                self.picam2.stop()
            except Exception as e:
                print("[Camera] Stop error:", e)
        self.picam2 = None

    def _capture_loop(self):
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration({"size": (self.width, self.height)})
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1.0)
            print("[Camera] Capture loop started.")
            while self.running:
                frame = self.picam2.capture_array("main")
                if frame is None:
                    time.sleep(0.05)
                    continue
                ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ret:
                    continue
                with self.frame_lock:
                    self.latest_frame = jpeg.tobytes()
                time.sleep(1.0 / self.fps)
        except Exception as e:
            print("[Camera] Error:", e)
        finally:
            if self.picam2:
                try:
                    self.picam2.stop()
                except:
                    pass
            print("[Camera] Capture thread exited.")

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame
