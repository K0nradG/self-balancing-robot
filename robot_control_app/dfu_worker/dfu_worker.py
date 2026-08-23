# Copyright 2026 Filip Dymczyk and Konrad Grucel

# DFU worker implementation that runs in a separate thread and handles DFU operations over BLE.

from PyQt6.QtCore import QThread, pyqtSignal

from dfu_utils.config import DEFAULT_BLE_TARGET
from dfu_utils.smpmgr_wrapper import check_smpmgr_available
from dfu_utils.firmware_utils import extract_firmware_from_zip
from .dfu_gui import DFU_GUI


class DFUWorker(QThread):
    progress_signal = pyqtSignal(int, str)  # (percent, description)
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(int, str)  # (return_code, message)

    def __init__(self, ble_target: str = DEFAULT_BLE_TARGET, parent=None):
        super().__init__(parent)
        self.ble_target = ble_target

    def run(self):
        is_available, error_message = check_smpmgr_available()
        if not is_available:
            self.finished_signal.emit(1, f"smpmgr unavailable: {error_message}")
            return

        try:
            self.log_signal.emit("Extracting firmware image...")
            firmware_image = extract_firmware_from_zip()

            dfu = DFU_GUI(
                ble_target=self.ble_target,
                image_path=firmware_image,
                progress_callback=lambda pct, msg: self.progress_signal.emit(pct, msg),
                log_callback=lambda msg: self.log_signal.emit(msg),
            )
            return_code, message = dfu.perform_update()
            self.finished_signal.emit(return_code, message)
        except Exception as e:
            self.finished_signal.emit(1, f"Execution error: {e}")
