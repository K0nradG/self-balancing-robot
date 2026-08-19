# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Robot Control application utilizing BLE NUS for communication with the self-balancing robot.

import logging
import time

from PyQt6.QtCore import QThread

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
from . import ble_protocol

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("NUS_App")


class RobotControlApp(QMainWindow):
    """Main Application Window acting as Coordinator/Mediator between Panels and Worker Threads."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Application")
        self.resize(1300, 800)

        self.is_connected = False
        self.dfu_skipped = False
        self.dfu_thread = None
        self._pending_dfu_action = None
        self._pending_dfu_target = None

        # Start BLE Background Thread
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

        # Lock control panel until DFU is completed/skipped
        self.right_panel.setEnabled(False)

    def __connect_signals(self):

        #  Left Panel signals connections
        self.left_panel.connect_requested.connect(self.start_connect)
        self.left_panel.disconnect_requested.connect(self.start_disconnect)
        self.left_panel.skip_dfu_requested.connect(self.handle_skip_dfu)
        self.left_panel.start_dfu_requested.connect(self.start_dfu_process)
        self.left_panel.auto_record_toggled.connect(
            self.ble_worker.set_auto_record
        )
        self.left_panel.enable_logs_toggled.connect(self.toggle_enable_logs)

        # Right panel signals connections
        self.right_panel.send_command_requested.connect(self.send_command)

        # BLE worker signals connections
        self.ble_worker.connected_signal.connect(self.update_connection_status)
        self.ble_worker.data_received_signal.connect(
            self.left_panel.display_received_data
        )
        self.ble_worker.battery_signal.connect(self.left_panel.update_battery_status)
        self.ble_worker.log_signal.connect(self.left_panel.log_message)
        self.ble_worker.telemetry_signal.connect(
            self.right_panel.update_telemetry_plots
        )
        self.ble_worker.pid_params_signal.connect(
            self.right_panel.update_pid_parameters
        )
        self.ble_worker.lqr_params_signal.connect(
            self.right_panel.update_lqr_parameters
        )
        self.ble_worker.command_result_signal.connect(self.handle_command_result)
        self.ble_worker.trajectory_complete_signal.connect(
            lambda: self.left_panel.log_message("Trajectory completed.")
        )
        self.ble_worker.app_version_signal.connect(
            lambda version: self.left_panel.log_message(
                "App version: "
                f"{version['major']}.{version['minor']}."
                f"{version['revision']}+{version['build']}"
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
        self.send_command(
            ble_protocol.dfu_command(ble_protocol.DfuAction.SKIP)
        )

    def start_dfu_process(self, target_name: str):
        if self.is_connected:
            self._pending_dfu_action = ble_protocol.DfuAction.START
            self._pending_dfu_target = target_name
            self.left_panel.set_dfu_running_state()
            self.left_panel.log_message(">> Requesting DFU mode...")
            self.send_command(
                ble_protocol.dfu_command(ble_protocol.DfuAction.START)
            )
            return

        self._launch_dfu(target_name)

    def _launch_dfu(self, target_name: str):
        if self.is_connected:
            self.left_panel.log_message(
                ">> Disconnecting active BLE session for DFU process..."
            )
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
            f"<< {request_type.name}: {status.name} "
            f"(request packet {result['request_packet_number']})"
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
            self.left_panel.log_message(
                ">> DFU skipped. Robot control panel unlocked."
            )
            self.send_command(ble_protocol.get_pid_state_command())
        elif pending_action == ble_protocol.DfuAction.START:
            target = self._pending_dfu_target
            self._pending_dfu_target = None
            if target:
                self._launch_dfu(target)

    def _on_dfu_finished(self, return_code: int, message: str):
        if return_code == 0:
            self.left_panel.log_message(
                f"<font color='green'><b>DFU Success:</b> {message}</font>"
            )
        else:
            self.left_panel.log_message(
                f"<font color='red'><b>DFU Failed (Code {return_code}):</b> {message}</font>"
            )
        self.left_panel.log_message("Connection can be restored")

        self.dfu_skipped = False
        self.update_connection_status(False, "")

    def toggle_enable_logs(self, checked: bool):
        self.ble_worker.enable_logs = checked
        status = "Enabled" if checked else "Disabled"
        self.left_panel.log_message(f">> BLE Logs {status}")

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
