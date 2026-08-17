# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Robot Control application utilizing BLE NUS for communication with the self-balancing robot. 

import logging
import time
from collections import deque

import pyqtgraph as pg
from PyQt6.QtCore import QThread
from PyQt6.QtGui import QDoubleValidator
from PyQt6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .ble_commands import *
from .ble_worker import BLEWorker
from .dfu_worker import DFUWorker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("NUS_App")

DEFAULT_CONNECT_TARGET_NAME = "SELF_BALANCING_ROBOT"
MAX_PLOT_POINTS = 200


class RobotControlApp(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Application")
        self.resize(1200, 720)

        self.is_connected = False
        self.dfu_skipped = False
        self.dfu_thread = None

        # Telemetry plot buffers
        self.plot_time = deque(maxlen=MAX_PLOT_POINTS)
        self.bs_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.ab_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.rs_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.ar_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.pwm0_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.pwm1_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.sample_idx = 0

        # Start BLE Background Thread
        self.ble_worker = BLEWorker()
        self.ble_worker.connected_signal.connect(self.update_connection_status)
        self.ble_worker.data_received_signal.connect(self.display_received_data)
        self.ble_worker.telemetry_signal.connect(self.update_telemetry_plots)
        self.ble_worker.log_signal.connect(self.log_message)
        self.ble_worker.start()

        self.__init_ui()

    def __init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        splitter = QSplitter()
        main_layout.addWidget(splitter)

        self.__create_left_ui_panel_widget()
        self.__create_right_ui_panel_widget()

        splitter.addWidget(self.left_widget)
        splitter.addWidget(self.right_widget)

        # Lock control panel by default until DFU is skipped
        self.right_widget.setEnabled(False)

    def __create_left_ui_panel_widget(self):
        self.left_widget = QWidget()
        left_layout = QVBoxLayout( self.left_widget)

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
        left_layout.addWidget(conn_group)

        self.dfu_group = QGroupBox("DFU")
        dfu_layout = QHBoxLayout(self.dfu_group)

        self.skip_btn = QPushButton("Skip DFU")
        self.skip_btn.setEnabled(False)
        self.skip_btn.clicked.connect(self.handle_skip_dfu)

        self.start_dfu_btn = QPushButton("Start DFU")
        self.start_dfu_btn.setEnabled(False)
        self.start_dfu_btn.clicked.connect(self.start_dfu_process)

        dfu_layout.addWidget(self.skip_btn)
        dfu_layout.addWidget(self.start_dfu_btn)
        left_layout.addWidget(self.dfu_group)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        left_layout.addWidget(self.console, stretch=1)

        input_group = QGroupBox("Command Interface")
        input_layout = QHBoxLayout(input_group)

        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command to send...")

        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.send_command_console)

        self.auto_rec_cb = QCheckBox("Auto-record Data")
        self.auto_rec_cb.setChecked(False)
        self.auto_rec_cb.toggled.connect(self.toggle_auto_record)

        input_layout.addWidget(self.cmd_input, stretch=2)
        input_layout.addWidget(self.send_btn)
        input_layout.addWidget(self.auto_rec_cb)
        left_layout.addWidget(input_group)

    def __create_right_ui_panel_widget(self):
        self.right_widget = QGroupBox("Robot Control Panel")
        right_layout = QVBoxLayout(self.right_widget)

        ctrl_actions_group = QGroupBox("Control Actions")
        ctrl_actions_layout = QVBoxLayout(ctrl_actions_group)

        btn_layout = QHBoxLayout()
        self.start_control_btn = QPushButton("Start Control")
        self.start_control_btn.clicked.connect(
            lambda: self.send_command(START_CONTROL)
        )
        self.stop_control_btn = QPushButton("Stop Control")
        self.stop_control_btn.clicked.connect(
            lambda: self.send_command(STOP_CONTROL)
        )
        btn_layout.addWidget(self.start_control_btn)
        btn_layout.addWidget(self.stop_control_btn)
        ctrl_actions_layout.addLayout(btn_layout)

        num_validator = QDoubleValidator(-180.0, 180.0, 4)
        num_validator.setNotation(QDoubleValidator.Notation.StandardNotation)

        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Distance Reference [m]:"))
        self.dist_ref_input = QLineEdit("0.0")
        self.dist_ref_input.setValidator(num_validator)
        dist_layout.addWidget(self.dist_ref_input)

        self.send_dist_btn = QPushButton("Set distance setpoint")
        self.send_dist_btn.clicked.connect(self.send_dist_setpoint)
        dist_layout.addWidget(self.send_dist_btn)

        ctrl_actions_layout.addLayout(dist_layout)

        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel("Rotation Angle Reference [deg]:"))
        self.rot_ref_input = QLineEdit("0.0")
        self.rot_ref_input.setValidator(num_validator)
        rot_layout.addWidget(self.rot_ref_input)

        self.send_rot_btn = QPushButton("Set rotation angle setpoint")
        self.send_rot_btn.clicked.connect(self.send_rot_setpoint)
        rot_layout.addWidget(self.send_rot_btn)

        ctrl_actions_layout.addLayout(rot_layout)
        right_layout.addWidget(ctrl_actions_group)

        pg.setConfigOption("background", "#1e1e1e")
        pg.setConfigOption("foreground", "#dcdcdc")

        self.bal_plot = pg.PlotWidget(title="Balance Angle")
        self.bal_plot.setLabel("left", "Angle [deg]")
        self.bal_plot.addLegend()
        self.curve_bs = self.bal_plot.plot(pen=pg.mkPen("c", width=2), name="Reference")
        self.curve_ab = self.bal_plot.plot(pen=pg.mkPen("m", width=2), name="Actual")
        right_layout.addWidget(self.bal_plot)

        self.rot_plot = pg.PlotWidget(title="Rotation Angle")
        self.rot_plot.setLabel("left", "Angle [deg]")
        self.rot_plot.addLegend()
        self.curve_rs = self.rot_plot.plot(pen=pg.mkPen("g", width=2), name="Reference")
        self.curve_ar = self.rot_plot.plot(pen=pg.mkPen("y", width=2), name="Actual")
        right_layout.addWidget(self.rot_plot)

        self.pwm_plot = pg.PlotWidget(title="Wheel PWMs")
        self.pwm_plot.setLabel("left", "PWM")
        self.pwm_plot.setLabel("bottom", "Samples")
        self.pwm_plot.addLegend()
        self.curve_pwm0 = self.pwm_plot.plot(pen=pg.mkPen("r", width=2), name="PWM0")
        self.curve_pwm1 = self.pwm_plot.plot(pen=pg.mkPen("b", width=2), name="PWM1")
        right_layout.addWidget(self.pwm_plot)

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

    def update_connection_status(self, connected: bool, address: str):
        self.is_connected = connected

        if connected:
            self.status_label.setText(f"Status: Connected ({address})")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.target_name_input.setEnabled(False)

            if not self.dfu_skipped:
                self.dfu_group.setEnabled(True)
                self.skip_btn.setEnabled(True)
                self.start_dfu_btn.setEnabled(True)
                self.right_widget.setEnabled(False)
            else:
                self.right_widget.setEnabled(True)
        else:
            # Reset DFU state upon disconnection so next connection forces DFU decisions again
            self.dfu_skipped = False
            self.status_label.setText("Status: Disconnected")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            
            # Re-enable DFU container for the next connection session
            self.dfu_group.setEnabled(True)
            self.skip_btn.setEnabled(False)
            self.start_dfu_btn.setEnabled(False)
            
            self.target_name_input.setEnabled(True)
            self.right_widget.setEnabled(False)

    def handle_skip_dfu(self):
        """Sends SKIP_DFU command, permanently locks DFU actions for the session, and unlocks Control Panel."""
        self.send_command(SKIP_DFU)
        self.dfu_skipped = True

        # Disable DFU buttons for this active session
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.dfu_group.setEnabled(False)

        # Unlock control panel
        self.right_widget.setEnabled(True)
        self.log_message(">> DFU Skipped. Robot control panel unlocked.")

    def start_dfu_process(self):
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
        self.log_message("Connection can be restored")

        self.dfu_skipped = False
        self.status_label.setText("Status: Disconnected")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.dfu_group.setEnabled(True)
        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.target_name_input.setEnabled(True)
        self.right_widget.setEnabled(False)

    def _set_dfu_running_state(self):
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(False)
        self.dfu_group.setEnabled(False)
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

    def send_dist_setpoint(self):

        dist_str = self.dist_ref_input.text().strip().replace(",", ".")
        try:
            dist_val = float(dist_str)
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Command rejected: Distance setpoint must be a valid number.",
                QMessageBox.StandardButton.Ok,
            )
            return

        self.send_command(f"{DISTANCE_SETPOINT}{dist_val:.2f}")

    def send_rot_setpoint(self):

        rot_str = self.rot_ref_input.text().strip().replace(",", ".")
        try:
            rot_val = float(rot_str)
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Command rejected: Rotation setpoint must be a valid number.",
                QMessageBox.StandardButton.Ok,
            )
            return

        self.send_command(f"{ROTATE_SETPOINT}{rot_val:.2f}")

    def send_command_console(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.ble_worker.send_command(cmd)
            self.cmd_input.clear()

    def send_command(self, text: str):
        if isinstance(text, str) and text.strip():
            self.ble_worker.send_command(text)


    def update_telemetry_plots(self, data: dict):
        self.sample_idx += 1
        self.plot_time.append(self.sample_idx)

        if "bs" in data:
            self.bs_buf.append(data["bs"])
        if "ab" in data:
            self.ab_buf.append(data["ab"])
        if "rs" in data:
            self.rs_buf.append(data["rs"])
        if "ar" in data:
            self.ar_buf.append(data["ar"])
        if "pwm0" in data:
            self.pwm0_buf.append(data["pwm0"])
        if "pwm1" in data:
            self.pwm1_buf.append(data["pwm1"])

        t_data = list(self.plot_time)
        if self.bs_buf:
            self.curve_bs.setData(t_data, list(self.bs_buf))
        if self.ab_buf:
            self.curve_ab.setData(t_data, list(self.ab_buf))
        if self.rs_buf:
            self.curve_rs.setData(t_data, list(self.rs_buf))
        if self.ar_buf:
            self.curve_ar.setData(t_data, list(self.ar_buf))
        if self.pwm0_buf:
            self.curve_pwm0.setData(t_data, list(self.pwm0_buf))
        if self.pwm1_buf:
            self.curve_pwm1.setData(t_data, list(self.pwm1_buf))

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