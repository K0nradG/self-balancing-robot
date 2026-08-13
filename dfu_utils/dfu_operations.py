#!/usr/bin/env python3

import time

from .config import (
    DEFAULT_POST_UPLOAD_DELAY,
    DEFAULT_REQUEST_TIMEOUT,
    DEFAULT_RETRIES,
    DEFAULT_RETRY_DELAY,
    DEFAULT_UPLOAD_TIMEOUT,
)
from .firmware_utils import format_duration, read_secondary_image_hash
from .progress_bar import ProgressBar
from .smpmgr_wrapper import run_smpmgr_with_retry


class DFUOperations:
    def __init__(
        self,
        ble_target,
        image_path,
        request_timeout=DEFAULT_REQUEST_TIMEOUT,
        upload_timeout=DEFAULT_UPLOAD_TIMEOUT,
        retries=DEFAULT_RETRIES,
        retry_delay=DEFAULT_RETRY_DELAY,
        post_upload_delay=DEFAULT_POST_UPLOAD_DELAY,
        no_confirm=False,
    ) -> None:

        self.ble_target = ble_target
        self.image_path = image_path
        self.request_timeout = request_timeout
        self.upload_timeout = upload_timeout
        self.retries = retries
        self.retry_delay = retry_delay
        self.post_upload_delay = post_upload_delay
        self.no_confirm = no_confirm

    def perform_update(self) -> tuple[int, str]:

        dfu_started = time.monotonic()
        self.progress = ProgressBar(
            total_steps=100, width=50, description="DFU Progress"
        )

        try:
            self.progress.update(0, "Checking current state...")
            command_arguments = ["image", "state-read"]
            run_smpmgr_with_retry(
                self.ble_target,
                self.request_timeout,
                command_arguments,
                retries=self.retries,
                retry_delay=self.retry_delay,
                capture_output=True,
            )
            self.progress.update(10, "Uploading image...")

            command_arguments = [
                "image",
                "upload",
                "--format",
                "any",
                str(self.image_path),
            ]
            run_smpmgr_with_retry(
                self.ble_target,
                self.upload_timeout,
                command_arguments,
                retries=self.retries,
                retry_delay=self.retry_delay,
                capture_output=True,
            )
            self.progress.update(60, "Reading image hash...")

            if self.post_upload_delay > 0:
                time.sleep(self.post_upload_delay)

            image_hash = read_secondary_image_hash(
                self.ble_target,
                self.request_timeout,
                retries=self.retries,
                retry_delay=self.retry_delay,
            )
            self.progress.update(70, "Scheduling test boot...")

            command_arguments = ["image", "state-write", image_hash]
            run_smpmgr_with_retry(
                self.ble_target,
                self.request_timeout,
                command_arguments,
                retries=self.retries,
                retry_delay=self.retry_delay,
                capture_output=True,
            )
            self.progress.update(80, "Resetting device...")

            command_arguments = ["os", "reset"]
            run_smpmgr_with_retry(
                self.ble_target,
                self.request_timeout,
                command_arguments,
                retries=self.retries,
                retry_delay=self.retry_delay,
                capture_output=True,
            )
            self.progress.update(85, "Waiting for device...")

            command_arguments = ["image", "state-read"]
            run_smpmgr_with_retry(
                self.ble_target,
                self.request_timeout,
                command_arguments,
                retries=self.retries,
                retry_delay=self.retry_delay,
                check=False,
                capture_output=True,
            )
            self.progress.update(95, "Confirming image...")

            if self.no_confirm:
                self.progress.update(100, "Complete!")
                self.progress.finish()
                elapsed = time.monotonic() - dfu_started
                return (
                    0,
                    f"Update completed in {format_duration(elapsed)}. Left in MCUboot test mode.",
                )

            command_arguments = ["image", "state-write", "--confirm"]
            run_smpmgr_with_retry(
                self.ble_target,
                self.request_timeout,
                command_arguments,
                retries=self.retries,
                retry_delay=self.retry_delay,
                capture_output=True,
            )

            self.progress.update(100, "Complete!")
            self.progress.finish()

            elapsed = time.monotonic() - dfu_started
            return (
                0,
                f"DFU update completed successfully in {format_duration(elapsed)}!",
            )

        except Exception as error:
            self.progress.finish()
            elapsed = time.monotonic() - dfu_started
            return 1, f"DFU failed: {str(error)}"
