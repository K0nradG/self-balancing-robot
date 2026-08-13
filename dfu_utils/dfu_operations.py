#!/usr/bin/env python3

import time
from pathlib import Path

from .config import (
    DEFAULT_POST_UPLOAD_DELAY, DEFAULT_RECONNECT_ATTEMPTS, DEFAULT_RECONNECT_DELAY,
    DEFAULT_REQUEST_TIMEOUT, DEFAULT_RETRIES, DEFAULT_RETRY_DELAY, DEFAULT_UPLOAD_TIMEOUT,
    PROGRESS_STEPS
)
from .firmware_utils import format_duration, read_secondary_image_hash
from .progress_bar import ProgressBar
from .smpmgr_wrapper import run_smpmgr_with_retry


class DFUOperations:
    def __init__(self, ble_target, image_path, request_timeout=DEFAULT_REQUEST_TIMEOUT,
                 upload_timeout=DEFAULT_UPLOAD_TIMEOUT, reconnect_attempts=DEFAULT_RECONNECT_ATTEMPTS,
                 reconnect_delay=DEFAULT_RECONNECT_DELAY, retries=DEFAULT_RETRIES,
                 retry_delay=DEFAULT_RETRY_DELAY, post_upload_delay=DEFAULT_POST_UPLOAD_DELAY,
                 no_confirm=False):
        self.ble_target = ble_target
        self.image_path = image_path
        self.request_timeout = request_timeout
        self.upload_timeout = upload_timeout
        self.reconnect_attempts = reconnect_attempts
        self.reconnect_delay = reconnect_delay
        self.retries = retries
        self.retry_delay = retry_delay
        self.post_upload_delay = post_upload_delay
        self.no_confirm = no_confirm
        
    def perform_update(self):
        dfu_started = time.monotonic()
        self.progress = ProgressBar(total_steps=100, width=50, description="DFU Progress")
        
        try:
            self.progress.update(0, "Checking current state...")
            run_smpmgr_with_retry(
                self.ble_target, self.request_timeout, "image", "state-read",
                retries=self.retries, retry_delay=self.retry_delay, capture_output=True
            )
            self.progress.update(10, "Uploading image...")
            
            run_smpmgr_with_retry(
                self.ble_target, self.upload_timeout, "image", "upload", "--format", "any",
                str(self.image_path), retries=self.retries, retry_delay=self.retry_delay,
                capture_output=True
            )
            self.progress.update(60, "Reading image hash...")
            
            if self.post_upload_delay > 0:
                time.sleep(self.post_upload_delay)
            
            image_hash = read_secondary_image_hash(
                self.ble_target, self.request_timeout,
                retries=self.retries, retry_delay=self.retry_delay
            )
            self.progress.update(70, "Scheduling test boot...")
            
            run_smpmgr_with_retry(
                self.ble_target, self.request_timeout, "image", "state-write", image_hash,
                retries=self.retries, retry_delay=self.retry_delay, capture_output=True
            )
            self.progress.update(80, "Resetting device...")
            
            run_smpmgr_with_retry(
                self.ble_target, self.request_timeout, "os", "reset",
                retries=self.retries, retry_delay=self.retry_delay, capture_output=True
            )
            self.progress.update(85, "Waiting for device...")
            
        except Exception as error:
            self.progress.finish()
            elapsed = time.monotonic() - dfu_started
            return 1, f"DFU failed: {str(error)}"
        
        if self.no_confirm:
            self.progress.update(100, "Complete!")
            self.progress.finish()
            elapsed = time.monotonic() - dfu_started
            return 0, f"Update completed in {format_duration(elapsed)}. Left in MCUboot test mode."
        
        reconnect_success = False
        for attempt in range(1, self.reconnect_attempts + 1):
            result = run_smpmgr_with_retry(
                self.ble_target, self.request_timeout, "image", "state-read",
                retries=1, check=False, capture_output=True
            )
            if result.returncode == 0:
                reconnect_success = True
                break
                
            if attempt < self.reconnect_attempts:
                time.sleep(self.reconnect_delay)
        
        if not reconnect_success:
            self.progress.finish()
            elapsed = time.monotonic() - dfu_started
            return 1, f"Device unreachable after {format_duration(elapsed)}. MCUboot rollback remains available."
        
        self.progress.update(95, "Confirming image...")
        
        try:
            run_smpmgr_with_retry(
                self.ble_target, self.request_timeout, "image", "state-write", "--confirm",
                retries=self.retries, retry_delay=self.retry_delay, capture_output=True
            )
        except Exception:
            self.progress.finish()
            elapsed = time.monotonic() - dfu_started
            return 1, f"Confirmation failed after {format_duration(elapsed)}. MCUboot rollback remains available."
        
        self.progress.update(100, "Complete!")
        self.progress.finish()
        
        elapsed = time.monotonic() - dfu_started
        return 0, f"DFU update completed successfully in {format_duration(elapsed)}!"

