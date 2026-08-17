# Copyright 2026 Filip Dymczyk and Konrad Grucel

# This is the BLE worker implementation that runs in a separate thread and handles BLE operations over NUS.

import asyncio
import csv
import logging
import os
import re
import time

from bleak import BleakClient, BleakScanner
from PyQt6.QtCore import QThread, pyqtSignal

NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Host -> Robot
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Robot -> Host

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("BLEWorker")

DEFAULT_LOG_DIR = "robot_data_logs"


class BLEWorker(QThread):
    connected_signal = pyqtSignal(bool, str)
    data_received_signal = pyqtSignal(str)
    telemetry_signal = pyqtSignal(dict)
    battery_signal = pyqtSignal(float)
    pid_params_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.loop = None
        self.client = None
        self.target_address = None
        self._notify_active = False
        self._expected_disconnect = False

        self.enable_logs = False
        self.auto_record = False
        self.csv_file = None
        self.csv_writer = None

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

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
            self.log_signal.emit(
                f"Device '{target_name}' not found within timeout."
            )
            self.connected_signal.emit(False, "")

    def _on_ble_disconnected(self, client):
        self._notify_active = False
        self._close_csv()

        if not self._expected_disconnect:
            self.log_signal.emit("Connection lost.")
            self.connected_signal.emit(False, "")

    async def _async_connect(self, address: str):
        self.target_address = address
        self._expected_disconnect = False
        try:
            self.client = BleakClient(
                self.target_address,
                disconnected_callback=self._on_ble_disconnected
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

        self._close_csv()
        self.connected_signal.emit(False, "")
        self.log_signal.emit("Disconnected from device.")

    def send_command(self, text: str):
        asyncio.run_coroutine_threadsafe(self._async_send(text), self.loop)

    async def _async_send(self, text: str):
        if self.client and self.client.is_connected:
            try:
                data = (text + "\n").encode("utf-8")
                await self.client.write_gatt_char(NUS_RX_CHAR_UUID, data)
                self.log_signal.emit(f">> Sent: {text}")
            except Exception as e:
                self.log_signal.emit(f"Send error: {e}")
        else:
            self.log_signal.emit("Cannot send: Not connected.")

    def _notification_handler(self, sender, data: bytearray):
        try:
            text = data.decode("utf-8", errors="replace").strip()

            if self.enable_logs:
                self.data_received_signal.emit(text)

            # 1. Parse telemetry key-value pairs
            parsed_telemetry = self._parse_telemetry(text)
            if parsed_telemetry:
                self.telemetry_signal.emit(parsed_telemetry)
                if self.auto_record:
                    self._record_telemetry(parsed_telemetry)
                if "bat_mv" in parsed_telemetry:
                    self.battery_signal.emit(parsed_telemetry["bat_mv"])

            # 2. Parse explicit battery text ("bat lvl mv 7953")
            bat_mv = self._parse_battery(text)
            if bat_mv is not None:
                self.battery_signal.emit(bat_mv)

            # 3. Parse PID parameters string
            pid_data = self._parse_pid_parameters(text)
            if pid_data:
                self.pid_params_signal.emit(pid_data)

        except Exception as e:
            logger.error(f"Error handling notification: {e}")

    def _parse_telemetry(self, text: str) -> dict:
        """Parses telemetry output string into a dictionary of floats."""
        matches = re.findall(r'([a-zA-Z0-9_]+):\s*(-?\d+(?:\.\d+)?)', text)
        if matches:
            return {key: float(val) for key, val in matches}
        return {}

    def _parse_battery(self, text: str) -> float | None:
        """Parses explicit battery level formatted as 'bat lvl mv 7953'."""
        match = re.search(r'bat\s+lvl\s+mv\s*:?\s*(\d+)', text, re.IGNORECASE)
        if match:
            return float(match.group(1))
        return None

    def _parse_pid_parameters(self, text: str) -> dict | None:
        """Parses PID values string and returns a mapped dictionary per controller."""
        pid_pattern = (
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)_"
            r"Kp([0-9.-]+)_Ki([0-9.-]+)_Kd([0-9.-]+)"
        )
        match = re.search(pid_pattern, text)
        if match:
            vals = [float(x) for x in match.groups()]
            controllers = ["distance", "linear_speed", "balance", "rotate", "wheel_speed"]
            return {
                ctrl_key: {
                    "kp": vals[i * 3],
                    "ki": vals[i * 3 + 1],
                    "kd": vals[i * 3 + 2],
                }
                for i, ctrl_key in enumerate(controllers)
            }
        return None

    def _record_telemetry(self, data: dict):
        if not os.path.exists(DEFAULT_LOG_DIR):
            os.makedirs(DEFAULT_LOG_DIR)

        if not self.csv_file:
            filename = time.strftime(f"{DEFAULT_LOG_DIR}/data_%Y%m%d_%H%M%S.csv")
            self.csv_file = open(filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            headers = ["timestamp"] + list(data.keys())
            self.csv_writer.writerow(headers)
            self.log_signal.emit(f"Started data logging: {filename}")

        row = [time.time()] + list(data.values())
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None