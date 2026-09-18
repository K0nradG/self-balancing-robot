# Copyright 2026 Filip Dymczyk and Konrad Grucel

# This is the BLE worker implementation that runs in a separate thread and handles BLE operations over NUS.

import asyncio
import logging
import time

from bleak import BleakClient, BleakScanner
from PyQt6.QtCore import QThread, pyqtSignal

from robot_control_app import ble_protocol

from .data_processor import DataProcessor, ParsedData

NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Host -> Robot
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Robot -> Host
PLOT_UPDATE_PERIOD_S = 0.02

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("BLEWorker")


class BLEWorker(QThread):
    connected_signal = pyqtSignal(bool, str)
    data_received_signal = pyqtSignal(str)
    telemetry_signal = pyqtSignal(dict)
    battery_signal = pyqtSignal(int, int)
    pid_params_signal = pyqtSignal(dict)
    lqr_params_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)
    app_version_signal = pyqtSignal(dict)
    trajectory_complete_signal = pyqtSignal()
    command_result_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        self.loop = None
        self.client = None
        self.target_address = None
        self._notify_active = False
        self._expected_disconnect = False
        self._last_telemetry_emit = 0.0
        self._write_queue = None
        self._write_task = None
        self._next_packet_number = 1

        self.enable_logs = False
        self.data_processor = DataProcessor()

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self._write_queue = asyncio.Queue()
        self._write_task = self.loop.create_task(self._write_loop())
        try:
            self.loop.run_forever()
        finally:
            self._write_task.cancel()
            self.loop.run_until_complete(
                asyncio.gather(self._write_task, return_exceptions=True)
            )
            self.data_processor.close_recording()
            self.loop.close()

    def set_auto_record(self, enabled: bool):
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.data_processor.set_recording, enabled)
        else:
            self.data_processor.set_recording(enabled)

    def scan_and_connect(self, target_name: str):
        asyncio.run_coroutine_threadsafe(
            self._async_scan_and_connect(target_name), self.loop
        )

    async def _async_scan_and_connect(self, target_name: str):
        self.log_signal.emit(f"Scanning for '{target_name}' (10s timeout)...")
        matched_device = None

        def detection_callback(device, advertisement_data):
            nonlocal matched_device
            name = device.name or advertisement_data.local_name or ""
            if target_name.lower() in name.lower() and matched_device is None:
                matched_device = device

        scanner = BleakScanner(detection_callback=detection_callback)
        await scanner.start()

        scan_time = 0.0
        while scan_time < 10.0 and matched_device is None:
            await asyncio.sleep(0.2)
            scan_time += 0.2

        await scanner.stop()

        if matched_device:
            self.log_signal.emit(
                f"Found device '{matched_device.name}' ({matched_device.address}). Connecting..."
            )
            await self._async_connect(matched_device.address)
        else:
            self.log_signal.emit(f"Device '{target_name}' not found within timeout.")
            self.connected_signal.emit(False, "")

    def _on_ble_disconnected(self, client):
        self._notify_active = False
        self.data_processor.close_recording()
        self.data_processor.reset_stream()
        self._discard_queued_writes()

        if not self._expected_disconnect:
            self.log_signal.emit("Connection lost.")
            self.connected_signal.emit(False, "")

    async def _async_connect(self, address: str):
        self.target_address = address
        self._expected_disconnect = False
        try:
            self.client = BleakClient(
                self.target_address,
                disconnected_callback=self._on_ble_disconnected,
            )
            await self.client.connect()
            if self.client.is_connected:
                await self.client.start_notify(
                    NUS_TX_CHAR_UUID, self._notification_handler
                )
                self._notify_active = True
                self.connected_signal.emit(True, self.target_address)
                self.log_signal.emit("Connected & Notifications enabled.")
            else:
                self.connected_signal.emit(False, "")
        except Exception as e:
            self.log_signal.emit(f"Connection failed: {e}")
            self.connected_signal.emit(False, "")

    def disconnect_device(self):
        asyncio.run_coroutine_threadsafe(self._async_disconnect(), self.loop)

    async def _async_disconnect(self):
        self._expected_disconnect = True
        if self.client and self.client.is_connected:
            try:
                if self._notify_active:
                    await self.client.stop_notify(NUS_TX_CHAR_UUID)
                await self.client.disconnect()
            except Exception as e:
                self.log_signal.emit(f"Disconnect error: {e}")

        self.data_processor.close_recording()
        self.data_processor.reset_stream()
        self._discard_queued_writes()
        self.connected_signal.emit(False, "")
        self.log_signal.emit("Disconnected from device.")

    def send_command(self, command: bytes):
        if not isinstance(command, bytes):
            self.log_signal.emit("Cannot send: command must be a binary RBT1 packet.")
            return
        try:
            packet = ble_protocol.unpack_packet(command)
        except ValueError as error:
            self.log_signal.emit(f"Cannot send invalid command: {error}")
            return

        if packet.message_type.value < ble_protocol.MessageType.STATE_COMMAND:
            self.log_signal.emit(
                f"Cannot send notification packet: {packet.message_type.name}"
            )
            return

        packet_number = self._next_packet_number
        self._next_packet_number = (packet_number + 1) & 0xFFFFFFFF
        queued_packet = ble_protocol.pack_packet(
            packet.message_type,
            packet.payload,
            packet_number=packet_number,
            reserved=packet.reserved,
        )
        if self.loop and self.loop.is_running() and self._write_queue is not None:
            self.loop.call_soon_threadsafe(self._write_queue.put_nowait, queued_packet)
        else:
            self.log_signal.emit("Cannot send: BLE worker is not running.")

    async def _write_loop(self):
        while True:
            command = await self._write_queue.get()
            try:
                if not self.client or not self.client.is_connected:
                    self.log_signal.emit("Cannot send: Not connected.")
                    continue
                await self.client.write_gatt_char(
                    NUS_RX_CHAR_UUID, command, response=True
                )
                packet = ble_protocol.unpack_packet(command)
                self.log_signal.emit(
                    f">> Sent {packet.message_type.name} (packet {packet.packet_number})"
                )
            except Exception as error:
                self.log_signal.emit(f"Send error: {error}")
            finally:
                self._write_queue.task_done()

    def _discard_queued_writes(self):
        if self._write_queue is None:
            return
        while True:
            try:
                self._write_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
            else:
                self._write_queue.task_done()

    def _notification_handler(self, sender, data: bytearray):
        try:
            packet = ble_protocol.unpack_packet(bytes(data))
            parsed: ParsedData = self.data_processor.process_packet(packet)

            if parsed.telemetry_samples:
                if self.enable_logs:
                    self.data_received_signal.emit(
                        f"TELEMETRY packet {packet.packet_number}: "
                        f"{len(parsed.telemetry_samples)} samples"
                    )
                now = time.monotonic()
                if (
                    parsed.telemetry is not None
                    and now - self._last_telemetry_emit >= PLOT_UPDATE_PERIOD_S
                ):
                    self._last_telemetry_emit = now
                    self.telemetry_signal.emit(parsed.telemetry)
            if parsed.log is not None:
                message = f"[{parsed.log.level}] {parsed.log.module}: {parsed.log.text}"
                self.log_signal.emit(message)
                if self.enable_logs:
                    self.data_received_signal.emit(message)
            if parsed.battery is not None:
                self.battery_signal.emit(
                    parsed.battery.millivolts, parsed.battery.percent
                )
            if parsed.app_version is not None:
                version = parsed.app_version
                self.app_version_signal.emit(
                    {
                        "major": version.major,
                        "minor": version.minor,
                        "revision": version.revision,
                        "build": version.build,
                    }
                )
            if parsed.pid_params is not None:
                self.pid_params_signal.emit(parsed.pid_params)
            if parsed.lqr_params is not None:
                self.lqr_params_signal.emit(parsed.lqr_params)
            if parsed.trajectory_complete:
                self.trajectory_complete_signal.emit()
            if parsed.command_result is not None:
                result = parsed.command_result
                self.command_result_signal.emit(
                    {
                        "request_packet_number": result.request_packet_number,
                        "request_type": result.request_type,
                        "status": result.status,
                    }
                )
            if parsed.identification_complete:
                self.log_signal.emit("Identification complete.")

        except ValueError as error:
            logger.warning("Rejected BLE notification: %s", error)
            self.log_signal.emit(f"Notification rejected: {error}")
        except Exception as error:
            logger.exception("Error handling notification")
            self.log_signal.emit(f"Notification error: {error}")
