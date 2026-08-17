# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Right panel widget handling robot control, control loop parameters and telemetry data plotting.


from robot_control_app.ble_commands import *

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QLineEdit,
    QGroupBox,
    QGridLayout,
    QMessageBox,
)
from PyQt6.QtCore import pyqtSignal
from collections import deque
import pyqtgraph as pg
from PyQt6.QtGui import QDoubleValidator

MAX_PLOT_POINTS = 200

PID_SET_COMMAND_MAP = {
    "distance": DISTANCE_PID,
    "linear_speed": LINEAR_SPEED_PID,
    "balance": BALANCE_PID,
    "rotate": ROTATE_PID,
    "wheel_speed": WHEEL_PID,
}


class RightPanelWidget(QWidget):
    """Handles Robot Control Actions, PID Parameter Tuning, and Real-time Telemetry Plots."""

    send_command_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
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

        self.__init_ui()

    def __init_ui(self):
        self.main_group = QGroupBox("Robot Control Panel")
        right_layout = QVBoxLayout(self)

        panel_layout = QVBoxLayout(self.main_group)

        ctrl_actions_group = QGroupBox("Control Actions")
        ctrl_actions_layout = QVBoxLayout(ctrl_actions_group)

        btn_layout = QHBoxLayout()
        self.start_control_btn = QPushButton("Start Control")
        self.start_control_btn.clicked.connect(
            lambda: self.send_command_requested.emit(START_CONTROL)
        )
        self.stop_control_btn = QPushButton("Stop Control")
        self.stop_control_btn.clicked.connect(
            lambda: self.send_command_requested.emit(STOP_CONTROL)
        )
        btn_layout.addWidget(self.start_control_btn)
        btn_layout.addWidget(self.stop_control_btn)
        ctrl_actions_layout.addLayout(btn_layout)

        num_validator = QDoubleValidator(-180.0, 180.0, 4)
        num_validator.setNotation(QDoubleValidator.Notation.StandardNotation)

        # Distance setpoint
        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Distance Reference [m]:"))
        self.dist_ref_input = QLineEdit("0.0")
        self.dist_ref_input.setValidator(num_validator)
        dist_layout.addWidget(self.dist_ref_input)

        self.send_dist_btn = QPushButton("Set distance setpoint")
        self.send_dist_btn.clicked.connect(self._send_dist_setpoint)
        dist_layout.addWidget(self.send_dist_btn)
        ctrl_actions_layout.addLayout(dist_layout)

        # Rotation setpoint
        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel("Rotation Angle Reference [deg]:"))
        self.rot_ref_input = QLineEdit("0.0")
        self.rot_ref_input.setValidator(num_validator)
        rot_layout.addWidget(self.rot_ref_input)

        self.send_rot_btn = QPushButton("Set rotation angle setpoint")
        self.send_rot_btn.clicked.connect(self._send_rot_setpoint)
        rot_layout.addWidget(self.send_rot_btn)

        ctrl_actions_layout.addLayout(rot_layout)
        panel_layout.addWidget(ctrl_actions_group)

        pid_group = self.__create_pid_tuning_group()
        panel_layout.addWidget(pid_group)

        pg.setConfigOption("background", "#1e1e1e")
        pg.setConfigOption("foreground", "#dcdcdc")

        self.bal_plot = pg.PlotWidget(title="Balance Angle")
        self.bal_plot.setLabel("left", "Angle [deg]")
        self.bal_plot.addLegend()
        self.curve_bs = self.bal_plot.plot(pen=pg.mkPen("c", width=2), name="Reference")
        self.curve_ab = self.bal_plot.plot(pen=pg.mkPen("m", width=2), name="Actual")
        panel_layout.addWidget(self.bal_plot)

        self.rot_plot = pg.PlotWidget(title="Rotation Angle")
        self.rot_plot.setLabel("left", "Angle [deg]")
        self.rot_plot.addLegend()
        self.curve_rs = self.rot_plot.plot(pen=pg.mkPen("g", width=2), name="Reference")
        self.curve_ar = self.rot_plot.plot(pen=pg.mkPen("y", width=2), name="Actual")
        panel_layout.addWidget(self.rot_plot)

        self.pwm_plot = pg.PlotWidget(title="Wheel PWMs")
        self.pwm_plot.setLabel("left", "PWM")
        self.pwm_plot.setLabel("bottom", "Samples")
        self.pwm_plot.addLegend()
        self.curve_pwm0 = self.pwm_plot.plot(pen=pg.mkPen("r", width=2), name="PWM0")
        self.curve_pwm1 = self.pwm_plot.plot(pen=pg.mkPen("b", width=2), name="PWM1")
        panel_layout.addWidget(self.pwm_plot)

        right_layout.addWidget(self.main_group)

    def __create_pid_tuning_group(self) -> QGroupBox:
        pid_group = QGroupBox("Control loop parameters")
        pid_layout = QVBoxLayout(pid_group)

        top_bar = QHBoxLayout()
        self.fetch_pid_btn = QPushButton("Get control loop parameters")
        self.fetch_pid_btn.clicked.connect(self._get_pid_parameters)
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
            set_btn.clicked.connect(lambda _, k=key: self._send_pid_parameters(k))

            grid.addWidget(kp_input, row, 1)
            grid.addWidget(ki_input, row, 2)
            grid.addWidget(kd_input, row, 3)
            grid.addWidget(set_btn, row, 4)

            self.pid_inputs[key] = {"kp": kp_input, "ki": ki_input, "kd": kd_input}

        pid_layout.addLayout(grid)
        return pid_group

    def _send_dist_setpoint(self):
        dist_str = self.dist_ref_input.text().strip().replace(",", ".")
        try:
            dist_val = float(dist_str)
            self.send_command_requested.emit(f"{DISTANCE_PID}{SETPOINT}{dist_val:.2f}")
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Command rejected: Distance setpoint must be a valid number.",
                QMessageBox.StandardButton.Ok,
            )

    def _send_rot_setpoint(self):
        rot_str = self.rot_ref_input.text().strip().replace(",", ".")
        try:
            rot_val = float(rot_str)
            self.send_command_requested.emit(f"{ROTATE_PID}{SETPOINT}{rot_val:.2f}")
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Command rejected: Rotation setpoint must be a valid number.",
                QMessageBox.StandardButton.Ok,
            )

    def _get_pid_parameters(self):
        self.send_command_requested.emit(GET_CONTROL_LOOP_PARAMS)

    def _send_pid_parameters(self, controller_key: str):
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

        cmd = f"{PID_SET_COMMAND_MAP[controller_key]}k{kp:.4f}i{ki:.4f}d{kd:.4f}"
        self.send_command_requested.emit(cmd)

    def update_pid_parameters(self, pid_data: dict):

        if not self.isEnabled():
            return

        for ctrl_key, params in pid_data.items():
            if ctrl_key in self.pid_inputs:
                self.pid_inputs[ctrl_key]["kp"].setText(f"{params['kp']:.4f}")
                self.pid_inputs[ctrl_key]["ki"].setText(f"{params['ki']:.4f}")
                self.pid_inputs[ctrl_key]["kd"].setText(f"{params['kd']:.4f}")

    def update_telemetry_plots(self, data: dict):

        if not self.isEnabled():
            return

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
