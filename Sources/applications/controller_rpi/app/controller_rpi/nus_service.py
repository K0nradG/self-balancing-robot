#!/usr/bin/env python3
import asyncio
import logging
import sys
from bleak import BleakClient

# Identification data
import time
import csv
import re
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("NUS")

# NUS UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> device
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # device -> host (notify)


class NUSClient:
    def __init__(self, address):
        self.address = address
        self.client = BleakClient(address)
        self.connected = False
        self._notify_active = False
        self.on_data = None
        self.device_name = None
        self.on_trajectory_ack = None

        # Identification recording data
        self.recording_started = False
        self.csv_file = None
        self.csv_writer = None
        self.last_ident_csv_path: Path | None = None

    IDENT_LINE_RE = re.compile(
        r"dt=(?P<dt>-?\d+(?:\.\d+)?)\s+"
        r"angle=(?P<angle>-?\d+(?:\.\d+)?)\s+"
        r"angle_dt=(?P<angle_dt>-?\d+(?:\.\d+)?)\s+"
        r"pwm=(?P<pwm>-?\d+(?:\.\d+)?)\s+"
        r"pos=(?P<pos>-?\d+(?:\.\d+)?)\s+"
        r"pos_dt=(?P<pos_dt>-?\d+(?:\.\d+)?)\s*$"
    )

    def _ensure_recording_started(self):

        if self.recording_started:
            return

        filename = time.strftime("ident_%Y%m%d_%H%M%S.csv")
        path = Path(filename)
        self.last_ident_csv_path = path

        self.csv_file = path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "dt", "angle", "angle_dt", "pwm", "pos", "pos_dt"])

        self.recording_started = True
        logger.info("AUTO-START identification recording: %s", path)

    def start_recording(self, filename: str | None = None):
        if self.recording:
            logger.warning("Recording already in progress")
            return

        if filename is None:
            filename = time.strftime("ident_%Y%m%d_%H%M%S.csv")

        path = Path(filename)
        self.csv_file = path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "dt", "angle", "angle_dt", "pwm"])

        self.recording = True
        logger.info("Recording to file: %s", path)

    def stop_recording(self):
        if not self.recording:
            logger.warning("Recording is not active")
            return

        self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.recording = False
        logger.info("Recording finished")

    async def connect(self):
        try:
            logger.info("Connecting to %s...", self.address)
            await self.client.connect()
            self.connected = self.client.is_connected if isinstance(self.client.is_connected, bool) else await self.client.is_connected()

            if self.connected:
                try:
                    self.device_name = await self.client.read_gatt_char("00002a00-0000-1000-8000-00805f9b34fb")
                    self.device_name = self.device_name.decode("utf-8", errors="ignore")
                    logger.info("Connected to %s (device name: %s)", self.address, self.device_name)
                except Exception:
                    self.device_name = None
                    logger.warning("Failed to read device name.")
            else:
                logger.error("Connection failed.")

        except Exception as e:
            logger.exception("Connection error: %s", e)

    async def disconnect(self):
        try:
            if self._notify_active:
                await self.notifications_off()

            await self.client.disconnect()
            self.connected = False
            logger.info("Disconnected")

            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                logger.info("CSV file closed")

        except Exception:
            pass

    async def send(self, data: str):
        if not self.connected:
            logger.warning("Client is not connected")
            return

        try:
            await self.client.write_gatt_char(NUS_RX_CHAR_UUID, data.encode("utf-8"))
            logger.info("Data sent: %r", data)
        except Exception as e:
            logger.exception("Send error: %s", e)

    async def notifications_on(self):
        if not self.connected:
            logger.warning("Client is not connected")
            return

        if self._notify_active:
            logger.info("Notifications already enabled")
            return

        try:
            await self.client.start_notify(NUS_TX_CHAR_UUID, self.notify_handler)
            self._notify_active = True
            logger.info("Notifications enabled")
        except Exception as e:
            logger.exception("Error enabling notifications: %s", e)

    async def notifications_off(self):
        if not self.connected:
            logger.warning("Client is not connected")
            return

        if not self._notify_active:
            logger.info("Notifications already disabled")
            return

        try:
            await self.client.stop_notify(NUS_TX_CHAR_UUID)
            self._notify_active = False
            logger.info("Notifications disabled")
        except Exception as e:
            logger.exception("Error disabling notifications: %s", e)

    def notify_handler(self, sender_handle, data: bytearray):
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            text = None

        hex_data = " ".join(f"{b:02x}" for b in data)
        logger.info("NOTIFY [%s]: hex=%s text=%r", sender_handle, hex_data, text)
        print(f"NOTIFY: {text}")

        if text:
            m = self.IDENT_LINE_RE.search(text)
            if m:
                self._ensure_recording_started()

                row = [
                    time.time(),
                    float(m["dt"]),
                    float(m["angle"]),
                    float(m["angle_dt"]),
                    float(m["pwm"]),
                    float(m["pos"]),
                    float(m["pos_dt"]),
                ]

                self.csv_writer.writerow(row)
                self.csv_file.flush()

        if not text:
            return

        if self.on_data:
            try:
                self.on_data(text)
            except Exception as e:
                logger.warning("Error in NUS on_data callback: %s", e)

        # special callback for watching trajectory acknowledgements
        if "tc" in text.lower():
            if self.on_trajectory_ack:
                try:
                    self.on_trajectory_ack(text)
                except Exception as e:
                    logger.warning("Error in NUS on_trajectory_ack callback: %s", e)

    def get_status(self) -> dict:
        return {
            "connected": self.connected,
            "notify_active": self._notify_active,
            "address": self.address,
            "device_name": self.device_name or "Unknown device"
        }


# Asynchronous input (non-blocking)
async def async_input(prompt: str = "") -> str:
    print(prompt, end="", flush=True)
    loop = asyncio.get_event_loop()
    return (await loop.run_in_executor(None, sys.stdin.readline)).rstrip("\n")


async def interactive_loop(client: NUSClient):
    print("Interactive mode. Commands:")
    print(" send \"data\"")
    print(" notifications on")
    print(" notifications off")
    print(" disconnect / exit")

    while True:
        cmd = await async_input("> ")
        cmd = cmd.strip()

        if cmd.startswith("send "):
            data = cmd[5:].strip('"')
            await client.send(data)

        elif cmd == "notifications on":
            await client.notifications_on()

        elif cmd == "notifications off":
            await client.notifications_off()

        elif cmd in ("disconnect", "exit"):
            await client.disconnect()
            break

        else:
            print("Unknown command. Available: send \"data\", notifications on, notifications off, disconnect")


async def main():
    import argparse

    parser = argparse.ArgumentParser(description="Interactive NUS client")
    parser.add_argument("--addr", "-a", required=True, help="BLE device MAC address")
    args = parser.parse_args()

    client = NUSClient(args.addr)
    await client.connect()

    if client.connected:
        await interactive_loop(client)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nTerminated")
