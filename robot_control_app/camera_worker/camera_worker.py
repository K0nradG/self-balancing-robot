import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QImage
import os

class CameraWorker(QThread):
    # Sygnały do komunikacji z głównym wątkiem aplikacji
    frame_signal = pyqtSignal(QImage)
    ai_drive_signal = pyqtSignal(float, float)  # angular_speed, linear_speed

    def __init__(self, rtsp_url="tcp://192.168.4.1:8555"):
        super().__init__()
        self.rtsp_url = rtsp_url
        self.running = True
        
        # Parametry sterowania (PD + skalowanie prędkości)
        self.base_linear_speed = 0.13
        self.kp_angular = -0.0105
        self.kd_angular = -0.002      # Dodany człon różniczkowy
        self.k_slow = 0.005           # Współczynnik zwalniania w zakrętach
        self.last_error = 0.0         # Pamięć błędu do obliczania pochodnej

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

            # 1. Konwersja na szarość i rozmycie
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 0)

            # 2. Binaryzacja
            _, mask = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY_INV)

            # 3. ROI (Region of Interest)
            mask[0:int(h/3), 0:w] = 0

            # 4. Operacje morfologiczne
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            angular = 0.0
            linear = 0.0

            if contours:
                valid_contours = [c for c in contours if cv2.contourArea(c) > 500]
                
                if valid_contours:
                    c = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(c)

                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Wizualizacja
                        cv2.drawContours(frame, [c], -1, (255, 0, 0), 2)
                        cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)

                        # Obliczenie błędu i jego pochodnej (PD)
                        error = (w / 2) - cx
                        error_derivative = error - self.last_error
                        self.last_error = error

                        angular = (self.kp_angular * error) + (self.kd_angular * error_derivative)
                        linear = self.base_linear_speed * max(0.2, 1.0 - min(1.0, self.k_slow * abs(error)))
                else:
                    self.last_error = 0.0
            else:
                self.last_error = 0.0

            self.ai_drive_signal.emit(angular, linear)

            # Konwersja do PyQt
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            bytes_per_line = 3 * w
            qt_img = QImage(
                rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
            )
            self.frame_signal.emit(qt_img.copy())

        cap.release()

    def stop(self):
        self.running = False
        self.wait()

