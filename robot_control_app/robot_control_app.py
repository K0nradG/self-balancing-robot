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
    QGridLayout,
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

PID_SET_COMMAND_MAP = {
    "distance": DISTANCE_PID,
    "linear_speed": LINEAR_SPEED_PID,
    "balance": BALANCE_PID,
    "rotate": ROTATE_PID,
    "wheel_speed": WHEEL_PID,
}


class RobotControlApp(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Application")
        self.resize(1300, 800)

        self.is_connected = False
        self.dfu_skipped = False
        self.dfu_thread = None

        # PID inputs structure map
        self.pid_inputs = {}

        # Telemetry plot buffers
        self.plot_time = deque(maxlen=MAX_PLOT_POINTS)
        self.bs_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.ab_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.rs_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.ar_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.pwm0_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.pwm1_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.sample_idx = 0

        # Start BLE Background Thread & Connect Signals
        self.ble_worker = BLEWorker()
        self.ble_worker.connected_signal.connect(self.update_connection_status)
        self.ble_worker.data_received_signal.connect(self.display_received_data)
        self.ble_worker.telemetry_signal.connect(self.update_telemetry_plots)
        self.ble_worker.battery_signal.connect(self.update_battery_status)
        self.ble_worker.pid_params_signal.connect(self.update_pid_parameters)
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
        left_layout = QVBoxLayout(self.left_widget)

        conn_group = QGroupBox("BLE Connection")
        conn_layout = QVBoxLayout(conn_group)

        # Top row: Connection controls
        conn_row1 = QHBoxLayout()
        self.target_name_input = QLineEdit(DEFAULT_CONNECT_TARGET_NAME)
        self.connect_btn = QPushButton("Scan && Connect")
        self.connect_btn.clicked.connect(self.start_connect)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.start_disconnect)
        self.disconnect_btn.setEnabled(False)

        self.status_label = QLabel("Status: Disconnected")

        conn_row1.addWidget(QLabel("Target Device Name:"))
        conn_row1.addWidget(self.target_name_input, stretch=2)
        conn_row1.addWidget(self.connect_btn)
        conn_row1.addWidget(self.disconnect_btn)
        conn_row1.addWidget(self.status_label, stretch=1)
        conn_layout.addLayout(conn_row1)

        # Bottom row: Battery Level LED & Status
        conn_row2 = QHBoxLayout()
        self.battery_led = QLabel()
        self.battery_led.setFixedSize(14, 14)
        self.set_battery_led("#888888")  # Initial gray state

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

        self.enable_logs_checkbox = QCheckBox("Enable BLE Logs")
        self.enable_logs_checkbox.setChecked(False)
        left_layout.addWidget(self.enable_logs_checkbox)
        self.enable_logs_checkbox.toggled.connect(self.toggle_enable_logs)

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

    def set_battery_led(self, color: str):
        """Helper method to update the CSS stylesheet of the circular LED widget."""
        self.battery_led.setStyleSheet(
            f"background-color: {color}; border-radius: 7px; border: 1px solid #444;"
        )

    def update_battery_status(self, mv: float):
        """Callback slot: Scales mV to Volts and updates the battery status LED/label."""
        volts = mv / 1000.0
        self.battery_label.setText(f"Battery Level: {volts:.2f} V")
        if volts >= 7.0:
            self.set_battery_led("#2ea44f")  # Green
        else:
            self.set_battery_led("#d73a49")  # Red

    def update_pid_parameters(self, pid_data: dict):
        """Callback slot: Updates PID line inputs from parsed controller dictionary."""
        for ctrl_key, params in pid_data.items():
            if ctrl_key in self.pid_inputs:
                self.pid_inputs[ctrl_key]["kp"].setText(f"{params['kp']:.4f}")
                self.pid_inputs[ctrl_key]["ki"].setText(f"{params['ki']:.4f}")
                self.pid_inputs[ctrl_key]["kd"].setText(f"{params['kd']:.4f}")

        self.log_message(">>PID Parameters parsed and updated in UI")

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

        pid_group = self.__create_pid_tuning_group()
        right_layout.addWidget(pid_group)

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

    def __create_pid_tuning_group(self) -> QGroupBox:
        pid_group = QGroupBox("Control loop parameters")
        pid_layout = QVBoxLayout(pid_group)

        top_bar = QHBoxLayout()
        self.fetch_pid_btn = QPushButton("Get control loop parameters")
        self.fetch_pid_btn.clicked.connect(self.get_pid_parameters)
        top_bar.addWidget(self.fetch_pid_btn)
        top_bar.addStretch()
        pid_layout.addLayout(top_bar)

        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Controller</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Kp</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Ki</b>"), 0, 2)
        grid.addWidget(QLabel("<b>Kd</b>"), 0, 3)
        grid.addWidget(QLabel("<b>Action</b>"), 0, 4)

        controllers = [
            ("distance", "Distance PID"),
            ("linear_speed", "Linear Speed PID"),
            ("balance", "Balance Angle PID"),
            ("rotate", "Rotation Angle PID"),
            ("wheel_speed", "Wheel Speed PID"),
        ]

        pid_validator = QDoubleValidator(-10000.0, 10000.0, 6)
        pid_validator.setNotation(QDoubleValidator.Notation.StandardNotation)

        for row, (key, label_text) in enumerate(controllers, start=1):
            grid.addWidget(QLabel(label_text), row, 0)

            kp_input = QLineEdit("0.0")
            kp_input.setValidator(pid_validator)
            ki_input = QLineEdit("0.0")
            ki_input.setValidator(pid_validator)
            kd_input = QLineEdit("0.0")
            kd_input.setValidator(pid_validator)

            set_btn = QPushButton(f"Set {label_text}")
            set_btn.clicked.connect(lambda _, k=key: self.send_pid_parameters(k))

            grid.addWidget(kp_input, row, 1)
            grid.addWidget(ki_input, row, 2)
            grid.addWidget(kd_input, row, 3)
            grid.addWidget(set_btn, row, 4)

            self.pid_inputs[key] = {"kp": kp_input, "ki": ki_input, "kd": kd_input}

        pid_layout.addLayout(grid)
        return pid_group

    def get_pid_parameters(self):
        self.send_command(GET_CONTROL_LOOP_PARAMS)
        self.log_message(">> Requested control loop PID parameters.")

    def send_pid_parameters(self, controller_key: str):
        inputs = self.pid_inputs.get(controller_key)
        if not inputs:
            return

        try:
            kp = float(inputs["kp"].text().strip().replace(",", "."))
            ki = float(inputs["ki"].text().strip().replace(",", "."))
            kd = float(inputs["kd"].text().strip().replace(",", "."))
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                f"Command rejected: Please enter valid numeric PID values for {controller_key}.",
                QMessageBox.StandardButton.Ok,
            )
            return

        self.send_command(f"{PID_SET_COMMAND_MAP[controller_key]}k{kp:.4f}i{ki:.4f}d{kd:.4f}")
        self.log_message(f">> Updated {controller_key} PID: Kp={kp}, Ki={ki}, Kd={kd}")

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
            self.dfu_skipped = False
            self.status_label.setText("Status: Disconnected")
            self.battery_label.setText("Battery Level: N/A")
            self.set_battery_led("#888888")

            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)

            self.dfu_group.setEnabled(True)
            self.skip_btn.setEnabled(False)
            self.start_dfu_btn.setEnabled(False)

            self.target_name_input.setEnabled(True)
            self.right_widget.setEnabled(False)

    def handle_skip_dfu(self):
        self.send_command(SKIP_DFU)
        self.dfu_skipped = True

        self.skip_btn.setEnabled(False)
        self.start_dfu_btn.setEnabled(False)
        self.dfu_group.setEnabled(False)

        self.right_widget.setEnabled(True)
        self.log_message(">> DFU Skipped. Robot control panel unlocked.")

        self.get_pid_parameters()

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
        self.battery_label.setText("Battery Level: N/A")
        self.set_battery_led("#888888")

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

        self.send_command(f"{DISTANCE_PID}{SETPOINT}{dist_val:.2f}")

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

        self.send_command(f"{ROTATE_PID}{SETPOINT}{rot_val:.2f}")

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

    def display_received_data(self, data: str):
        """Callback slot: Logs raw incoming BLE string to the console text box."""
        self.console.append(f"<font color='green'>&lt;&lt; {data}</font>")

    def log_message(self, msg):
        self.console.append(f"<i>{msg}</i>")

    def toggle_auto_record(self, checked):
        self.ble_worker.auto_record = checked

    def toggle_enable_logs(self, checked):
        self.ble_worker.enable_logs = checked
        if checked:
            self.log_message(">> BLE Logs Enabled")
        else:
            self.log_message(">> BLE Logs Disabled")

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