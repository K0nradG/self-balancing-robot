# Copyright 2026 Filip Dymczyk and Konrad Grucel

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QLineEdit,
    QGroupBox,
    QCheckBox,
    QTextEdit,
    QStackedWidget,
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap

DEFAULT_CONNECT_TARGET_NAME = "SELF_BALANCING_ROBOT"


class LeftPanelWidget(QWidget):

    connect_requested = pyqtSignal(str)
    disconnect_requested = pyqtSignal()
    skip_dfu_requested = pyqtSignal()
    start_dfu_requested = pyqtSignal(str)
    auto_record_toggled = pyqtSignal(bool)
    enable_logs_toggled = pyqtSignal(bool)
    toggle_camera_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.__init_ui()

    def __init_ui(self):
        left_layout = QVBoxLayout(self)

        conn_group = QGroupBox("BLE Connection")
        conn_layout = QVBoxLayout(conn_group)

        conn_row1 = QHBoxLayout()
        self.target_name_input = QLineEdit(DEFAULT_CONNECT_TARGET_NAME)
        self.connect_btn = QPushButton("Scan && Connect")
        self.connect_btn.clicked.connect(self.__on_connect_clicked)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_requested.emit)
        self.disconnect_btn.setEnabled(False)

        self.status_label = QLabel("Status: Disconnected")

        conn_row1.addWidget(QLabel("Target Device Name:"))
        conn_row1.addWidget(self.target_name_input, stretch=2)
        conn_row1.addWidget(self.connect_btn)
        conn_row1.addWidget(self.disconnect_btn)
        conn_row1.addWidget(self.status_label, stretch=1)
        conn_layout.addLayout(conn_row1)

        conn_row2 = QHBoxLayout()
        self.battery_led = QLabel()
        self.battery_led.setFixedSize(14, 14)
        self.set_battery_led("#888888")

        self.battery_label = QLabel("Battery Level: N/A")
        conn_row2.addWidget(QLabel("Battery Status:"))
        conn_row2.addWidget(self.battery_led)
        conn_row2.addWidget(self.battery_label)
        conn_row2.addStretch()
        conn_layout.addLayout(conn_row2)

        left_layout.addWidget(conn_group)

        self.dfu_group = QGroupBox("DFU")
        dfu_layout = QHBoxLayout(self.dfu_group)

        self.skip_btn = QPushButton("Skip DFU")
        self.skip_btn.setEnabled(False)
        self.skip_btn.clicked.connect(self.skip_dfu_requested.emit)

        self.start_dfu_btn = QPushButton("Start DFU")
        self.start_dfu_btn.setEnabled(False)
        self.start_dfu_btn.clicked.connect(self.__on_start_dfu_clicked)

        dfu_layout.addWidget(self.skip_btn)
        dfu_layout.addWidget(self.start_dfu_btn)
        left_layout.addWidget(self.dfu_group)

        camera_group = QGroupBox("Camera Stream")
        camera_layout = QVBoxLayout(camera_group)
        self.toggle_camera_btn = QPushButton("Show Camera View")
        self.toggle_camera_btn.setCheckable(True)
        self.toggle_camera_btn.clicked.connect(self.toggle_camera_requested.emit)
        camera_layout.addWidget(self.toggle_camera_btn)
        left_layout.addWidget(camera_group)

        # Użycie QStackedWidget do przełączania między konsolą logów a widokiem kamery
        self.stack = QStackedWidget()

        # Strona 0: Konsola tekstowa
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.stack.addWidget(self.console)

        # Strona 1: Etykieta wideo z kamery
        self.camera_label = QLabel("Camera Feed Disconnected")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        self.stack.addWidget(self.camera_label)

        left_layout.addWidget(self.stack, stretch=1)

        self.enable_logs_checkbox = QCheckBox("Enable BLE Logs")
        self.enable_logs_checkbox.setChecked(False)
        self.enable_logs_checkbox.toggled.connect(self.enable_logs_toggled.emit)
        left_layout.addWidget(self.enable_logs_checkbox)

        input_group = QGroupBox("Telemetry Recording")
        input_layout = QHBoxLayout(input_group)

        self.auto_rec_cb = QCheckBox("Auto-record Data")
        self.auto_rec_cb.setChecked(False)
        self.auto_rec_cb.toggled.connect(self.auto_record_toggled.emit)

        input_layout.addWidget(self.auto_rec_cb)
        input_layout.addStretch()
        left_layout.addWidget(input_group)

    def __on_connect_clicked(self):
        target = self.target_name_input.text().strip()
        if target:
            self.connect_requested.emit(target)
        else:
            self.log_message("Error: Target device name cannot be empty.")

    def __on_start_dfu_clicked(self):
        target = self.target_name_input.text().strip()
        if target:
            self.start_dfu_requested.emit(target)
        else:
            self.log_message("Error: Device target name is required for DFU.")

    def set_camera_mode(self, shown: bool):
        """Przełącza widok w stosie: pokazaj kamerę (strona 1) lub konsolę (strona 0)."""
        if shown:
            self.toggle_camera_btn.setText("Hide Camera View")
            self.toggle_camera_btn.setChecked(True)
            self.stack.setCurrentIndex(1)
        else:
            self.toggle_camera_btn.setText("Show Camera View")
            self.toggle_camera_btn.setChecked(False)
            self.stack.setCurrentIndex(0)

    def update_camera_frame(self, image: QImage):
        """Skaluje i wyświetla klatkę wideo w miejscu konsoli."""
        pixmap = QPixmap.fromImage(image)
        scaled_pixmap = pixmap.scaled(
            self.camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.camera_label.setPixmap(scaled_pixmap)

    # --- UI Update Slots & Public Methods ---
    def set_battery_led(self, color: str):
        self.battery_led.setStyleSheet(
            f"background-color: {color}; border-radius: 7px; border: 1px solid #444;"
        )

    def update_battery_status(self, mv: int, percent: int):
        volts = mv / 1000.0
        self.battery_label.setText(f"Battery Level: {volts:.2f} V ({percent}%)")
        self.set_battery_led("#2ea44f" if volts >= 7.0 else "#d73a49")

    def update_connection_status(
        self, connected: bool, address: str, dfu_skipped: bool
    ):
        if connected:
            self.status_label.setText(f"Status: Connected ({address})")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.target_name_input.setEnabled(False)

            if not dfu_skipped:
                self.dfu_group.setEnabled(True)
                self.skip_btn.setEnabled(True)
                self.start_dfu_btn.setEnabled(True)
        else:
            self.status_label.setText("Status: Disconnected")
            self.battery_label.setText("Battery Level: N/A")
            self.set_battery_led("#888888")

            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.dfu_group.setEnabled(True)
            self.skip_btn.setEnabled(False)
            self.start_dfu_btn.setEnabled(False)
            self.target_name_input.setEnabled(True)

    def set_connecting_state(self):
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(False)
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.target_name_input.setEnabled(False)
        self.status_label.setText("Status: Connecting...")

    def set_dfu_running_state(self):
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(False)
        self.dfu_group.setEnabled(False)
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.target_name_input.setEnabled(False)
        self.status_label.setText("Status: DFU In Progress...")

    def set_dfu_skipped_state(self):
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.dfu_group.setEnabled(False)

    def display_dfu_progress(self, percent: int, description: str):
        self.console.append(f"<b>[DFU {percent}%]</b> {description}")

    def display_received_data(self, data: str):
        self.console.append(f"<font color='green'>&lt;&lt; {data}</font>")

    def log_message(self, msg: str):
        self.console.append(f"<i>{msg}</i>")

