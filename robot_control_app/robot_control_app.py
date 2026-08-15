# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Robot Control application utilizing BLE NUS for communication with the self-balancing robot. 


import logging
import time

from PyQt6.QtCore import QThread
from PyQt6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .ble_worker import BLEWorker
from .dfu_worker import DFUWorker

from .ble_commands import *

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("NUS_App")

DEFAULT_CONNECT_TARGET_NAME = "SELF_BALANCING_ROBOT"


class RobotControlApp(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Application")
        self.resize(750, 620)

        self.is_connected = False
        self.dfu_thread = None

        # Start BLE Background Thread
        self.ble_worker = BLEWorker()
        self.ble_worker.connected_signal.connect(self.update_connection_status)
        self.ble_worker.data_received_signal.connect(self.display_received_data)
        self.ble_worker.log_signal.connect(self.log_message)
        self.ble_worker.start()

        self._init_ui()

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # --- BLE Connection Group ---
        conn_group = QGroupBox("BLE Connection")
        conn_layout = QHBoxLayout(conn_group)

        self.target_name_input = QLineEdit(DEFAULT_CONNECT_TARGET_NAME)

        self.connect_btn = QPushButton("Scan && Connect")
        self.connect_btn.clicked.connect(self.start_connect)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.start_disconnect)
        self.disconnect_btn.setEnabled(False)

        self.status_label = QLabel("Status: Disconnected")

        conn_layout.addWidget(QLabel("Target Device Name:"))
        conn_layout.addWidget(self.target_name_input, stretch=2)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        conn_layout.addWidget(self.status_label, stretch=1)
        layout.addWidget(conn_group)

        # --- DFU Group ---
        dfu_group = QGroupBox("DFU")
        dfu_layout = QHBoxLayout(dfu_group)

        self.skip_btn = QPushButton("Skip DFU")
        self.skip_btn.setEnabled(False)
        self.skip_btn.clicked.connect(lambda: self.send_command(SKIP_DFU))

        self.start_dfu_btn = QPushButton("Start DFU")
        self.start_dfu_btn.setEnabled(False)  # Ready when disconnected
        self.start_dfu_btn.clicked.connect(self.start_dfu_process)

        dfu_layout.addWidget(self.skip_btn)
        dfu_layout.addWidget(self.start_dfu_btn)
        layout.addWidget(dfu_group)

        # --- Console Output ---
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        layout.addWidget(self.console, stretch=1)

        # --- Command Interface Group ---
        input_group = QGroupBox("Command Interface")
        input_layout = QHBoxLayout(input_group)

        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command to send...")
        self.cmd_input.returnPressed.connect(self.send_command_console)

        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.send_command)

        self.auto_rec_cb = QCheckBox("Auto-record Telemetry")
        self.auto_rec_cb.setChecked(True)
        self.auto_rec_cb.toggled.connect(self.toggle_auto_record)

        input_layout.addWidget(self.cmd_input, stretch=2)
        input_layout.addWidget(self.send_btn)
        input_layout.addWidget(self.auto_rec_cb)
        layout.addWidget(input_group)

    def start_connect(self):
        target_name = self.target_name_input.text().strip()
        if not target_name:
            self.log_message("Error: Target device name cannot be empty.")
            return

        self._set_controls_enabled(connecting=True)
        self.status_label.setText("Status: Connecting...")
        self.ble_worker.scan_and_connect(target_name)

    def start_disconnect(self):
        self._set_controls_enabled(connecting=True)
        self.ble_worker.disconnect_device()

    def update_connection_status(self, connected, address):
        self.is_connected = connected

        if connected:
            self.status_label.setText(f"Status: Connected ({address})")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.skip_btn.setEnabled(True)
            self.start_dfu_btn.setEnabled(True)
            self.target_name_input.setEnabled(False)
        else:
            self.status_label.setText("Status: Disconnected")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.skip_btn.setEnabled(False)
            self.start_dfu_btn.setEnabled(False)
            self.target_name_input.setEnabled(True)

    def start_dfu_process(self):
        """Disconnects BLE if connected, locks controls, and starts DFUWorker thread."""
        target_name = self.target_name_input.text().strip()
        if not target_name:
            self.log_message("Error: Device target name is required for DFU.")
            return

        if self.is_connected:
            self.log_message(">> Disconnecting active BLE session for DFU process...")
            self.ble_worker.disconnect_device()
            QThread.msleep(500)

        self._set_dfu_running_state()

        self.log_message(">> Starting DFU worker thread...")
        self.dfu_thread = DFUWorker(ble_target=target_name)
        self.dfu_thread.progress_signal.connect(self.display_dfu_progress)
        self.dfu_thread.log_signal.connect(self.log_message)
        self.dfu_thread.finished_signal.connect(self._on_dfu_finished)
        self.dfu_thread.start()

    def display_dfu_progress(self, percent: int, description: str):
        self.console.append(f"<b>[DFU {percent}%]</b> {description}")

    def _on_dfu_finished(self, return_code: int, message: str):
        if return_code == 0:
            self.log_message(f"<font color='green'><b>DFU Success:</b> {message}</font>")
        else:
            self.log_message(f"<font color='red'><b>DFU Failed (Code {return_code}):</b> {message}</font>")
        self.log_message(f"Connection can be restored")

        # Restore disconnected state controls
        self.status_label.setText("Status: Disconnected")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.target_name_input.setEnabled(True)

    def _set_dfu_running_state(self):
        """Disables connect and DFU buttons while DFU runs."""
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(False)
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.target_name_input.setEnabled(False)
        self.status_label.setText("Status: DFU In Progress...")

    def _set_controls_enabled(self, connecting: bool):
        self.connect_btn.setEnabled(not connecting)
        self.disconnect_btn.setEnabled(not connecting)
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(not connecting)
        self.target_name_input.setEnabled(not connecting)

    def send_command_console(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.ble_worker.send_command(cmd)
            self.cmd_input.clear()

    def send_command(self, text: str):
        self.ble_worker.send_command(text)

    def display_received_data(self, data):
        self.console.append(f"<font color='green'>&lt;&lt; {data}</font>")

    def log_message(self, msg):
        self.console.append(f"<i>{msg}</i>")

    def toggle_auto_record(self, checked):
        self.ble_worker.auto_record = checked

    def closeEvent(self, event):
        if self.dfu_thread and self.dfu_thread.isRunning():
            self.dfu_thread.quit()
            self.dfu_thread.wait(1000)

        if self.is_connected:
            self.ble_worker.disconnect_device()
            time.sleep(0.3)

        if self.ble_worker.loop:
            self.ble_worker.loop.call_soon_threadsafe(self.ble_worker.loop.stop)
        self.ble_worker.quit()
        self.ble_worker.wait(1000)
        event.accept()