import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QImage
import os

class CameraWorker(QThread):
    frame_signal = pyqtSignal(QImage)
    line_tracking_signal = pyqtSignal(float, bool)

    def __init__(self, rtsp_url="tcp://192.168.4.1:8555"):
        super().__init__()
        self.rtsp_url = rtsp_url
        self.running = True

    def run(self):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        while self.running:
            ret, frame = cap.read()
            if not ret:
                self.msleep(20)
                continue

            h, w, _ = frame.shape
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 0)
            _, mask = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY_INV)
            mask[0:int(h/3), 0:w] = 0

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            line_found = False
            error = 0.0

            if contours:
                valid_contours = [c for c in contours if cv2.contourArea(c) > 500]
                if valid_contours:
                    c = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        cv2.drawContours(frame, [c], -1, (255, 0, 0), 2)
                        cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
                        
                        error = (w / 2) - cx
                        line_found = True

            self.line_tracking_signal.emit(error, line_found)

            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            bytes_per_line = 3 * w
            qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            self.frame_signal.emit(qt_img.copy())

        cap.release()

    def stop(self):
        self.running = False
        self.wait()

