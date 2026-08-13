#!/usr/bin/env python3

import subprocess
import sys
import time
import shutil
from pathlib import Path

from .config import DEFAULT_RETRIES, DEFAULT_RETRY_DELAY, SMPMGR_PATH


def run_smpmgr(ble_target, timeout, *arguments, check=True, capture_output=False):
    executable = str(SMPMGR_PATH) if sys.platform == "win32" else SMPMGR_PATH
    command = [executable, "--timeout", str(timeout), "--ble", ble_target, *arguments]
    
    if capture_output:
        stdout = subprocess.PIPE
        stderr = subprocess.STDOUT
    else:
        stdout = subprocess.DEVNULL
        stderr = subprocess.DEVNULL
    
    result = subprocess.run(
        command,
        check=False,
        stdout=stdout,
        stderr=stderr,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    
    if check and result.returncode != 0:
        raise subprocess.CalledProcessError(result.returncode, command)
    
    return result


def run_smpmgr_with_retry(ble_target, timeout, *arguments, retries=DEFAULT_RETRIES,
                          retry_delay=DEFAULT_RETRY_DELAY, check=True, capture_output=False):
    last_error = None
    
    for attempt in range(1, retries + 1):
        try:
            result = run_smpmgr(ble_target, timeout, *arguments,
                               check=check, capture_output=capture_output)
            return result
        except subprocess.CalledProcessError as error:
            last_error = error
            if attempt < retries:
                time.sleep(retry_delay)
            else:
                raise
    
    if last_error:
        raise last_error
    
    raise RuntimeError("Unexpected retry loop exit")


def check_smpmgr_available():
    if sys.platform == "win32":
        if not Path(SMPMGR_PATH).is_file():
            return False, f"smpmgr executable not found at {SMPMGR_PATH}"
    else:
        if shutil.which("smpmgr") is None:
            return False, "smpmgr is not installed or is not in PATH"
    
    return True, ""

