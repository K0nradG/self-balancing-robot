import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QImage

class CameraWorker(QThread):
    # Sygnały do komunikacji z głównym wątkiem aplikacji
    frame_signal = pyqtSignal(QImage)
    ai_drive_signal = pyqtSignal(float, float)  # angular_speed, linear_speed

    def __init__(self, rtsp_url="rtsp://192.168.4.1:8554/camera"):
        super().__init__()
        self.rtsp_url = rtsp_url
        self.running = True
        
        # Parametry naszego "modelu" podążania za linią
        self.base_linear_speed = 0.13  # Zmniejszona prędkość do przodu
        self.kp_angular = -0.0105         # Zmniejszona agresywność skrętu

    def run(self):
        cap = cv2.VideoCapture(self.rtsp_url)
        
        # Pętla musi być z wcięciem (wewnątrz funkcji run)
        while self.running:
            ret, frame = cap.read()
            if not ret:
                self.msleep(20)
                continue

            h, w, _ = frame.shape

            # 1. Konwersja na szarość i rozmycie (usuwa szumy i fakturę podłogi)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 0)

            # 2. Binaryzacja (dostosuj wartość 70 jeśli taśma nie jest wykrywana)
            _, mask = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY_INV)

            # 3. ROI (Region of Interest) - ignorujemy górną połowę ekranu (patrzymy pod nogi)
            mask[0:int(h/2), 0:w] = 0

            # 4. Operacje morfologiczne (usuwają małe plamki z paneli)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            angular = 0.0
            linear = 0.0

            if contours:
                # 5. Filtrowanie i wybór - odrzucamy zbyt małe śmieci
                valid_contours = [c for c in contours if cv2.contourArea(c) > 500]
                
                if valid_contours:
                    c = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(c)

                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Wizualizacja
                        cv2.drawContours(frame, [c], -1, (255, 0, 0), 2)  # Obrys taśmy
                        cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)   # Środek taśmy
                        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 2) # Środek kamery

                        # Obliczenie błędu
                        error = (w / 2) - cx
                        angular = self.kp_angular * error
                        linear = self.base_linear_speed

            self.ai_drive_signal.emit(angular, linear)

            # Konwersja do PyQt
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            bytes_per_line = 3 * w
            qt_img = QImage(
                rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
            )
            self.frame_signal.emit(qt_img.copy())

        # cap.release() musi być poza pętlą while! Wywoła się dopiero przy wyłączaniu AI
        cap.release()

    def stop(self):
        self.running = False
        self.wait()
