from robot_control_app import ble_protocol
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
    QComboBox,
    QCheckBox,
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QDoubleValidator, QPixmap, QImage
from collections import deque
import pyqtgraph as pg

MAX_PLOT_POINTS = 200

CONTROLLER_MAP = {
    "distance": ble_protocol.ControllerId.DISTANCE,
    "linear_speed": ble_protocol.ControllerId.LINEAR_SPEED,
    "balance": ble_protocol.ControllerId.BALANCE,
    "rotate": ble_protocol.ControllerId.ROTATE,
    "wheel_speed": ble_protocol.ControllerId.WHEEL_SPEED,
}


class RightPanelWidget(QWidget):
    send_command_requested = pyqtSignal(bytes)
    keyboard_control_toggled = pyqtSignal(bool)
    ai_control_toggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pid_inputs = {}
        self.balance_uses_lqr = False
        self.robot_running = False
        self.current_mode = ble_protocol.ControlMode.STANDARD

        self.plot_time = deque(maxlen=MAX_PLOT_POINTS)
        self.bs_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.ab_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.rs_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.ar_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.pwm0_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.pwm1_buf = deque(maxlen=MAX_PLOT_POINTS)
        self.sample_idx = 0

        self.__init_ui()
        self._update_mode_visibility()

    def __init_ui(self):
        self.main_group = QGroupBox("Robot Control Panel")
        right_layout = QVBoxLayout(self)
        panel_layout = QVBoxLayout(self.main_group)

        ctrl_actions_group = QGroupBox("Control Actions")
        ctrl_actions_layout = QVBoxLayout(ctrl_actions_group)

        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("<b>Control Mode:</b>"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Standard", ble_protocol.ControlMode.STANDARD)
        self.mode_combo.addItem("Free Drive", ble_protocol.ControlMode.FREE_DRIVE)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        ctrl_actions_layout.addLayout(mode_layout)

        btn_layout = QHBoxLayout()
        self.start_control_btn = QPushButton("Start Control")
        self.start_control_btn.clicked.connect(self._on_start_clicked)
        self.stop_control_btn = QPushButton("Stop Control")
        self.stop_control_btn.clicked.connect(self._on_stop_clicked)
        btn_layout.addWidget(self.start_control_btn)
        btn_layout.addWidget(self.stop_control_btn)
        ctrl_actions_layout.addLayout(btn_layout)

        num_validator = QDoubleValidator(-180.0, 180.0, 4)
        num_validator.setNotation(QDoubleValidator.Notation.StandardNotation)

        self.standard_widget = QWidget()
        std_layout = QVBoxLayout(self.standard_widget)
        std_layout.setContentsMargins(0, 0, 0, 0)

        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Distance Reference [m]:"))
        self.dist_ref_input = QLineEdit("0.0")
        self.dist_ref_input.setValidator(num_validator)
        dist_layout.addWidget(self.dist_ref_input)
        self.send_dist_btn = QPushButton("Set distance setpoint")
        self.send_dist_btn.clicked.connect(self._send_dist_setpoint)
        dist_layout.addWidget(self.send_dist_btn)
        std_layout.addLayout(dist_layout)

        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel("Rotation Angle Reference [deg]:"))
        self.rot_ref_input = QLineEdit("0.0")
        self.rot_ref_input.setValidator(num_validator)
        rot_layout.addWidget(self.rot_ref_input)
        self.send_rot_btn = QPushButton("Set rotation angle setpoint")
        self.send_rot_btn.clicked.connect(self._send_rot_setpoint)
        rot_layout.addWidget(self.send_rot_btn)
        std_layout.addLayout(rot_layout)

        ctrl_actions_layout.addWidget(self.standard_widget)

        # Free Drive Widget z podglądem kamery i przełącznikami
        self.drive_widget = QWidget()
        drive_layout = QVBoxLayout(self.drive_widget)
        drive_layout.setContentsMargins(0, 0, 0, 0)

        controls_layout = QHBoxLayout()

        self.kb_control_cb = QCheckBox("Enable Keyboard Control (Arrows)")
        self.kb_control_cb.toggled.connect(self.keyboard_control_toggled.emit)
        controls_layout.addWidget(self.kb_control_cb)

        self.ai_control_cb = QCheckBox("Enable Autonomous Control")
        self.ai_control_cb.toggled.connect(self.ai_control_toggled.emit)
        controls_layout.addWidget(self.ai_control_cb)

        drive_layout.addLayout(controls_layout)

        # Ekran podglądu z kamery
        self.video_label = QLabel("Camera feed disabled")
        self.video_label.setMinimumSize(320, 240)
        self.video_label.setStyleSheet("background-color: black; color: white; border: 1px solid #444;")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        drive_layout.addWidget(self.video_label)

        ctrl_actions_layout.addWidget(self.drive_widget)

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

    def __create_pid_tuning_group(self):
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

    def update_camera_feed(self, image: QImage):
        """Aktualizuje podgląd wideo w interfejsie."""
        if self.isVisible() and self.drive_widget.isVisible():
            pixmap = QPixmap.fromImage(image).scaled(
                400, 300, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
            )
            self.video_label.setPixmap(pixmap)

    def _on_mode_changed(self, index):
        new_mode = self.mode_combo.itemData(index)
        if new_mode == self.current_mode:
            return

        if self.robot_running:
            QMessageBox.warning(
                self,
                "Cannot change mode",
                "Stop the robot before changing control mode.",
                QMessageBox.StandardButton.Ok,
            )
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(self.mode_combo.findData(self.current_mode))
            self.mode_combo.blockSignals(False)
            return

        self.current_mode = new_mode
        self._update_mode_visibility()

        if new_mode == ble_protocol.ControlMode.STANDARD:
            self.send_command_requested.emit(
                ble_protocol.set_mode_command(ble_protocol.ControlMode.STANDARD)
            )
        elif new_mode == ble_protocol.ControlMode.FREE_DRIVE:
            self.send_command_requested.emit(
                ble_protocol.set_mode_command(ble_protocol.ControlMode.FREE_DRIVE)
            )

    def _update_mode_visibility(self):
        is_standard = self.current_mode == ble_protocol.ControlMode.STANDARD
        self.standard_widget.setVisible(is_standard)
        self.drive_widget.setVisible(not is_standard)
        self.rot_plot.setVisible(is_standard)

    def _on_start_clicked(self):
        self.robot_running = True
        self.mode_combo.setEnabled(False)
        self.send_command_requested.emit(
            ble_protocol.state_command(ble_protocol.StateAction.START)
        )

    def _on_stop_clicked(self):
        self.robot_running = False
        self.mode_combo.setEnabled(True)
        self.send_command_requested.emit(
            ble_protocol.state_command(ble_protocol.StateAction.STOP)
        )

    def _send_dist_setpoint(self):
        dist_str = self.dist_ref_input.text().strip().replace(",", ".")
        try:
            dist_val = float(dist_str)
            self.send_command_requested.emit(
                ble_protocol.set_setpoint_command(
                    ble_protocol.ControllerId.DISTANCE, dist_val
                )
            )
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
            self.send_command_requested.emit(
                ble_protocol.set_setpoint_command(
                    ble_protocol.ControllerId.ROTATE, rot_val
                )
            )
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Command rejected: Rotation setpoint must be a valid number.",
                QMessageBox.StandardButton.Ok,
            )

    def _get_pid_parameters(self):
        self.send_command_requested.emit(ble_protocol.get_pid_state_command())

    def _send_pid_parameters(self, controller_key):
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

        if controller_key == "balance" and self.balance_uses_lqr:
            self.send_command_requested.emit(ble_protocol.set_lqr_command(kp, ki))
        else:
            controller_id = CONTROLLER_MAP[controller_key]
            self.send_command_requested.emit(
                ble_protocol.set_pid_command(controller_id, kp, ki, kd)
            )

    def update_pid_parameters(self, pid_data):
        if not self.isEnabled():
            return
        for ctrl_key, params in pid_data.items():
            if ctrl_key in self.pid_inputs:
                self.pid_inputs[ctrl_key]["kp"].setText(f"{params['kp']:.4f}")
                self.pid_inputs[ctrl_key]["ki"].setText(f"{params['ki']:.4f}")
                self.pid_inputs[ctrl_key]["kd"].setText(f"{params['kd']:.4f}")

    def update_lqr_parameters(self, lqr_data):
        self.balance_uses_lqr = True
        inputs = self.pid_inputs["balance"]
        inputs["kp"].setText(f"{lqr_data['kx']:.4f}")
        inputs["ki"].setText(f"{lqr_data['ky']:.4f}")
        inputs["kd"].setText("0.0000")
        inputs["kd"].setEnabled(False)

    def update_telemetry_plots(self, data):
        if not self.isEnabled():
            return

        self.sample_idx += 1
        self.plot_time.append(self.sample_idx)

        self.bs_buf.append(data["bs"])
        self.ab_buf.append(data["ab"])
        self.rs_buf.append(data["rs"])
        self.ar_buf.append(data["ar"])
        self.pwm0_buf.append(data["pwm0"])
        self.pwm1_buf.append(data["pwm1"])

        t_data = list(self.plot_time)
        self.curve_bs.setData(t_data, list(self.bs_buf))
        self.curve_ab.setData(t_data, list(self.ab_buf))
        self.curve_rs.setData(t_data, list(self.rs_buf))
        self.curve_ar.setData(t_data, list(self.ar_buf))
        self.curve_pwm0.setData(t_data, list(self.pwm0_buf))
        self.curve_pwm1.setData(t_data, list(self.pwm1_buf))
    
