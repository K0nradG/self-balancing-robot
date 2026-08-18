# Copyright 2026 Filip Dymczyk and Konrad Grucel

# GUI for the DFU process.

import time

from dfu_utils.config import (
    DEFAULT_POST_UPLOAD_DELAY,
    DEFAULT_RECONNECT_ATTEMPTS,
    DEFAULT_RECONNECT_DELAY,
    DEFAULT_REQUEST_TIMEOUT,
    DEFAULT_RETRIES,
    DEFAULT_RETRY_DELAY,
    DEFAULT_UPLOAD_TIMEOUT,
)
from dfu_utils.firmware_utils import format_duration, read_secondary_image_hash
from .smpmngr_wrapper_gui import run_smpmgr_with_retry_stream


class DFU_GUI:
    def __init__(
        self,
        ble_target: str,
        image_path,
        request_timeout=DEFAULT_REQUEST_TIMEOUT,
        upload_timeout=DEFAULT_UPLOAD_TIMEOUT,
        retries=DEFAULT_RETRIES,
        retry_delay=DEFAULT_RETRY_DELAY,
        post_upload_delay=DEFAULT_POST_UPLOAD_DELAY,
        reconnect_attempts=DEFAULT_RECONNECT_ATTEMPTS,
        reconnect_delay=DEFAULT_RECONNECT_DELAY,
        progress_callback=None,
        log_callback=None,
    ) -> None:
        self.ble_target = ble_target
        self.image_path = image_path
        self.request_timeout = request_timeout
        self.upload_timeout = upload_timeout
        self.retries = retries
        self.retry_delay = retry_delay
        self.post_upload_delay = post_upload_delay
        self.reconnect_attempts = reconnect_attempts
        self.reconnect_delay = reconnect_delay
        self.progress_callback = progress_callback
        self.log_callback = log_callback

    def _notify_progress(self, percent: int, msg: str):
        if self.progress_callback:
            self.progress_callback(percent, msg)

    def perform_update(self) -> tuple[int, str]:
        dfu_started = time.monotonic()

        try:
            self._notify_progress(0, "Checking current device state...")
            run_smpmgr_with_retry_stream(
                self.ble_target,
                self.request_timeout,
                ["image", "state-read"],
                retries=self.retries,
                retry_delay=self.retry_delay,
            )

            self._notify_progress(10, "Uploading image...")
            run_smpmgr_with_retry_stream(
                self.ble_target,
                self.upload_timeout,
                ["image", "upload", "--format", "any", str(self.image_path)],
                retries=self.retries,
                retry_delay=self.retry_delay,
                progress_callback=self.progress_callback,
            )

            self._notify_progress(60, "Reading secondary image hash...")
            if self.post_upload_delay > 0:
                time.sleep(self.post_upload_delay)

            image_hash = read_secondary_image_hash(
                self.ble_target,
                self.request_timeout,
                retries=self.retries,
                retry_delay=self.retry_delay,
            )

            self._notify_progress(70, "Scheduling test boot...")
            run_smpmgr_with_retry_stream(
                self.ble_target,
                self.request_timeout,
                ["image", "state-write", image_hash],
                retries=self.retries,
                retry_delay=self.retry_delay,
            )

            self._notify_progress(80, "Resetting device...")
            run_smpmgr_with_retry_stream(
                self.ble_target,
                self.request_timeout,
                ["os", "reset"],
                retries=self.retries,
                retry_delay=self.retry_delay,
            )

            self._notify_progress(85, "Waiting for device reboot...")

            reconnect_success = False
            command_arguments = ["image", "state-read"]
            for attempt in range(1, self.reconnect_attempts + 1):
                return_code = run_smpmgr_with_retry_stream(
                    self.ble_target,
                    self.request_timeout,
                    command_arguments,
                    retries=1,
                    check=False,
                )
                if return_code == 0:
                    reconnect_success = True
                    break

                if attempt < self.reconnect_attempts:
                    time.sleep(self.reconnect_delay)

            if not reconnect_success:
                self.progress.finish()
                elapsed = time.monotonic() - dfu_started
                return (
                    1,
                    f"Device unreachable after {format_duration(elapsed)}. MCUboot rollback remains available.",
                )

            self._notify_progress(95, "Confirming image...")
            run_smpmgr_with_retry_stream(
                self.ble_target,
                self.request_timeout,
                ["image", "state-write", "--confirm"],
                retries=self.retries,
                retry_delay=self.retry_delay,
            )

            self._notify_progress(100, "DFU Complete!")
            elapsed = time.monotonic() - dfu_started
            return 0, f"DFU completed successfully in {format_duration(elapsed)}!"

        except Exception as error:
            elapsed = time.monotonic() - dfu_started
            return 1, f"DFU failed after {format_duration(elapsed)}: {str(error)}"
