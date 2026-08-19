# Copyright 2026 Filip Dymczyk and Konrad Grucel

import asyncio
import importlib.util
import sys
import types
import unittest

from robot_control_app import ble_protocol

if importlib.util.find_spec("bleak") is None:
    bleak_module = types.ModuleType("bleak")
    bleak_module.BleakClient = object
    bleak_module.BleakScanner = object
    sys.modules["bleak"] = bleak_module

if importlib.util.find_spec("PyQt6") is None:
    class DummySignal:
        def emit(self, *args):
            pass

    qt_core_module = types.ModuleType("PyQt6.QtCore")
    qt_core_module.QThread = object
    qt_core_module.pyqtSignal = lambda *args: DummySignal()
    pyqt_module = types.ModuleType("PyQt6")
    pyqt_module.QtCore = qt_core_module
    sys.modules["PyQt6"] = pyqt_module
    sys.modules["PyQt6.QtCore"] = qt_core_module

from robot_control_app.ble_worker.ble_worker import BLEWorker, NUS_RX_CHAR_UUID


class FakeBleakClient:
    def __init__(self):
        self.is_connected = True
        self.calls = []
        self.active_writes = 0
        self.max_active_writes = 0

    async def write_gatt_char(self, uuid, data, response):
        self.active_writes += 1
        self.max_active_writes = max(self.max_active_writes, self.active_writes)
        await asyncio.sleep(0)
        self.calls.append((uuid, data, response))
        self.active_writes -= 1


class BLEWorkerTest(unittest.IsolatedAsyncioTestCase):
    async def test_write_queue_serializes_binary_commands_with_response(self):
        worker = BLEWorker()
        worker._write_queue = asyncio.Queue()
        worker.client = FakeBleakClient()
        writer = asyncio.create_task(worker._write_loop())
        commands = (
            ble_protocol.state_command(
                ble_protocol.StateAction.START, sequence=1
            ),
            ble_protocol.get_pid_state_command(sequence=2),
        )

        try:
            for command in commands:
                await worker._write_queue.put(command)
            await asyncio.wait_for(worker._write_queue.join(), timeout=1)
        finally:
            writer.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await writer

        self.assertEqual(worker.client.max_active_writes, 1)
        self.assertEqual(
            worker.client.calls,
            [
                (NUS_RX_CHAR_UUID, commands[0], True),
                (NUS_RX_CHAR_UUID, commands[1], True),
            ],
        )


if __name__ == "__main__":
    unittest.main()
