# Copyright 2026 Filip Dymczyk and Konrad Grucel

import logging
import time

from PyQt6.QtCore import QThread, pyqtSignal, QUrl, QTimer, Qt
from PyQt6.QtGui import QImage
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QMainWindow,
    QSplitter,
    QWidget,
)

from .widgets.left_panel_widget import LeftPanelWidget
from .widgets.right_panel_widget import RightPanelWidget
from .ble_worker.ble_worker import BLEWorker
from .dfu_worker.dfu_worker import DFUWorker
from .camera_worker.camera_worker import CameraWorker
from .self_drive_control.self_drive_control import SelfDriveController
from . import ble_protocol

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("NUS_App")


class RobotControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Application")
        self.resize(1300, 800)

        self.is_connected = False
        self.dfu_skipped = False
        self.dfu_thread = None
        self._pending_dfu_action = None
        self._pending_dfu_target = None

        self.pressed_keys = set()
        self.keyboard_control_enabled = False
        
        self.self_drive_enabled = False
        self.self_drive_angular = 0.0
        self.self_drive_linear = 0.0
        self.camera_worker = None
        self.self_drive_controller = SelfDriveController()

        self.configured_linear_speed = 2.0
        self.configured_angular_speed = 19.0

        self.drive_timer = QTimer(self)
        self.drive_timer.timeout.connect(self._send_current_drive_command)
        self.drive_timer.start(100)

        self.ble_worker = BLEWorker()
        self.ble_worker.start()

        self.__init_ui()
        self.__connect_signals()

    def __init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        splitter = QSplitter()
        main_layout.addWidget(splitter)

        self.left_panel = LeftPanelWidget()
        self.right_panel = RightPanelWidget()

        splitter.addWidget(self.left_panel)
        splitter.addWidget(self.right_panel)

        self.right_panel.setEnabled(False)

    def __connect_signals(self):
        self.left_panel.connect_requested.connect(self.start_connect)
        self.left_panel.disconnect_requested.connect(self.start_disconnect)
        self.left_panel.skip_dfu_requested.connect(self.handle_skip_dfu)
        self.left_panel.start_dfu_requested.connect(self.start_dfu_process)
        self.left_panel.auto_record_toggled.connect(self.ble_worker.set_auto_record)
        self.left_panel.enable_logs_toggled.connect(self.toggle_enable_logs)

        self.right_panel.send_command_requested.connect(self.send_command)
        self.right_panel.keyboard_control_toggled.connect(self.set_keyboard_control)
        self.right_panel.ai_control_toggled.connect(self.set_self_drive_control)

        self.ble_worker.connected_signal.connect(self.update_connection_status)
        self.ble_worker.data_received_signal.connect(self.left_panel.display_received_data)
        self.ble_worker.battery_signal.connect(self.left_panel.update_battery_status)
        self.ble_worker.log_signal.connect(self.left_panel.log_message)
        self.ble_worker.telemetry_signal.connect(self.right_panel.update_telemetry_plots)
        self.ble_worker.pid_params_signal.connect(self.right_panel.update_pid_parameters)
        self.ble_worker.lqr_params_signal.connect(self.right_panel.update_lqr_parameters)
        self.ble_worker.command_result_signal.connect(self.handle_command_result)
        self.ble_worker.app_version_signal.connect(
            lambda version: self.left_panel.log_message(
                f"App version: {version['major']}.{version['minor']}.{version['revision']}+{version['build']}"
            )
        )

    def start_connect(self, target_name: str):
        self.left_panel.set_connecting_state()
        self.ble_worker.scan_and_connect(target_name)

    def start_disconnect(self):
        self.left_panel.set_connecting_state()
        self.ble_worker.disconnect_device()

    def send_command(self, command: bytes):
        if isinstance(command, bytes):
            self.ble_worker.send_command(command)

    def update_connection_status(self, connected: bool, address: str):
        self.is_connected = connected
        self.left_panel.update_connection_status(connected, address, self.dfu_skipped)

        if connected:
            self.right_panel.setEnabled(self.dfu_skipped)
        else:
            self.dfu_skipped = False
            self.right_panel.setEnabled(False)

    def handle_skip_dfu(self):
        self._pending_dfu_action = ble_protocol.DfuAction.SKIP
        self.left_panel.set_dfu_skipped_state()
        self.left_panel.log_message(">> Waiting for robot control initialization...")
        self.send_command(ble_protocol.dfu_command(ble_protocol.DfuAction.SKIP))

    def start_dfu_process(self, target_name: str):
        if self.is_connected:
            self._pending_dfu_action = ble_protocol.DfuAction.START
            self._pending_dfu_target = target_name
            self.left_panel.set_dfu_running_state()
            self.left_panel.log_message(">> Requesting DFU mode...")
            self.send_command(ble_protocol.dfu_command(ble_protocol.DfuAction.START))
            return

        self._launch_dfu(target_name)

    def _launch_dfu(self, target_name: str):
        if self.is_connected:
            self.left_panel.log_message(">> Disconnecting active BLE session for DFU process...")
            self.ble_worker.disconnect_device()
            QThread.msleep(500)
            
        self.left_panel.set_dfu_running_state()
        self.left_panel.log_message(">> Starting DFU worker thread...")

        self.dfu_thread = DFUWorker(ble_target=target_name)
        self.dfu_thread.progress_signal.connect(self.left_panel.display_dfu_progress)
        self.dfu_thread.log_signal.connect(self.left_panel.log_message)
        self.dfu_thread.finished_signal.connect(self._on_dfu_finished)
        self.dfu_thread.start()

    def handle_command_result(self, result: dict):
        request_type = result["request_type"]
        status = result["status"]
        self.left_panel.log_message(
            f"<< {request_type.name}: {status.name} (request packet {result['request_packet_number']})"
        )

        if request_type != ble_protocol.MessageType.DFU_COMMAND:
            return

        pending_action = self._pending_dfu_action
        self._pending_dfu_action = None
        
        if status != ble_protocol.CommandStatus.OK:
            self.left_panel.update_connection_status(
                self.is_connected, self.ble_worker.target_address or "", False
            )
            return

        if pending_action == ble_protocol.DfuAction.SKIP:
            self.dfu_skipped = True
            self.left_panel.set_dfu_skipped_state()
            self.right_panel.setEnabled(True)
            self.left_panel.log_message(">> DFU skipped. Robot control panel unlocked.")
            self.send_command(ble_protocol.get_pid_state_command())
        elif pending_action == ble_protocol.DfuAction.START:
            target = self._pending_dfu_target
            self._pending_dfu_target = None
            if target:
                self._launch_dfu(target)

    def _on_dfu_finished(self, return_code: int, message: str):
        if return_code == 0:
            self.left_panel.log_message(f"<font color='green'><b>DFU Success:</b> {message}</font>")
        else:
            self.left_panel.log_message(f"<font color='red'><b>DFU Failed (Code {return_code}):</b> {message}</font>")
            
        self.left_panel.log_message("Connection can be restored")
        self.dfu_skipped = False
        self.update_connection_status(False, "")

    def toggle_enable_logs(self, checked: bool):
        self.ble_worker.enable_logs = checked
        status = "Enabled" if checked else "Disabled"
        self.left_panel.log_message(f">> BLE Logs {status}")

    def set_self_drive_control(self, enabled: bool):
        self.self_drive_enabled = enabled
        if enabled:
            if self.keyboard_control_enabled:
                self.right_panel.kb_control_cb.setChecked(False)

            self.self_drive_controller.reset()
            self.camera_worker = CameraWorker()
            self.camera_worker.frame_signal.connect(self.right_panel.update_camera_feed)
            self.camera_worker.line_tracking_signal.connect(self._process_self_drive_perception)
            self.camera_worker.start()
            self.left_panel.log_message(">> Autonomous self drive mode started.")
        else:
            if self.camera_worker:
                self.camera_worker.stop()
                self.camera_worker = None
                self.right_panel.video_label.setText("Camera feed stopped")
                self.self_drive_angular = 0.0
                self.self_drive_linear = 0.0
            self.left_panel.log_message(">> Autonomous self drive mode stopped.")

    def _process_self_drive_perception(self, error: float, line_found: bool):
        angular, linear = self.self_drive_controller.calculate_speeds(error, line_found)
        self.self_drive_angular = angular
        self.self_drive_linear = linear

    def set_keyboard_control(self, enabled: bool):
        self.keyboard_control_enabled = enabled
        if enabled:
            if self.self_drive_enabled:
                self.right_panel.ai_control_cb.setChecked(False)

            self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
            self.setFocus()
        else:
            self.pressed_keys.clear()
            self._send_current_drive_command()

    def _send_current_drive_command(self):
        if not self.is_connected:
            return

        if self.self_drive_enabled:
            cmd = ble_protocol.drive_command(self.self_drive_angular, self.self_drive_linear)
            self.send_command(cmd)
            return

        if not self.keyboard_control_enabled:
            return

        linear = 0.0
        angular = 0.0

        keys = self.pressed_keys

        is_up = Qt.Key.Key_Up in keys or Qt.Key.Key_Up.value in keys
        is_down = Qt.Key.Key_Down in keys or Qt.Key.Key_Down.value in keys
        is_left = Qt.Key.Key_Left in keys or Qt.Key.Key_Left.value in keys
        is_right = Qt.Key.Key_Right in keys or Qt.Key.Key_Right.value in keys

        if is_up:
            linear += self.configured_linear_speed
        if is_down:
            linear -= self.configured_linear_speed

        if is_up:
            if is_left:
                angular -= self.configured_angular_speed
            elif is_right:
                angular += self.configured_angular_speed
        elif is_down:
            if is_left:
                angular += self.configured_angular_speed
            elif is_right:
                angular -= self.configured_angular_speed
        else:
            if is_left:
                angular -= self.configured_angular_speed
            elif is_right:
                angular += self.configured_angular_speed

        cmd = ble_protocol.drive_command(angular, linear)
        self.send_command(cmd)

    def keyPressEvent(self, event):
        if not self.keyboard_control_enabled or event.isAutoRepeat():
            super().keyPressEvent(event)
            return

        if event.key() in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
            self.pressed_keys.add(event.key())
            self._send_current_drive_command()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not self.keyboard_control_enabled or event.isAutoRepeat():
            super().keyReleaseEvent(event)
            return

        if event.key() in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
            self.pressed_keys.discard(event.key())
            self._send_current_drive_command()
        else:
            super().keyReleaseEvent(event)

    def closeEvent(self, event):
        if self.camera_worker and self.camera_worker.isRunning():
            self.camera_worker.stop()

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