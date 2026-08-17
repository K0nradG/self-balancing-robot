# Copyright 2026 Filip Dymczyk and Konrad Grucel

# SMPMGR wrapper implementation that runs in a separate thread and handles DFU operations over BLE.


import re
import subprocess
import sys
import time

from dfu_utils.config import DEFAULT_RETRIES, DEFAULT_RETRY_DELAY, SMPMGR_PATH


def run_smpmgr_stream(
    ble_target: str,
    timeout: float,
    command_arguments: list,
    log_callback=None,
    progress_callback=None,
    check: bool = True,
) -> str:
    """Runs smpmgr and streams stdout line-by-line to GUI callbacks."""
    executable = str(SMPMGR_PATH) if sys.platform == "win32" else SMPMGR_PATH
    command = [executable, "--timeout", str(timeout), "--ble", ble_target] + command_arguments

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )

    output_lines = []
    while True:
        line = process.stdout.readline()
        if not line and process.poll() is not None:
            break
        if line:
            clean_line = line.strip()
            output_lines.append(clean_line)

            if log_callback and clean_line:
                log_callback(clean_line)

            # Extract upload progress percentage if emitted by smpmgr
            match = re.search(r"(\d+)\s*%", clean_line)
            if match and progress_callback:
                pct = int(match.group(1))
                mapped_pct = 10 + int(pct * 0.5)  # Scale upload step to 10% - 60%
                progress_callback(mapped_pct, f"Uploading firmware ({pct}%)...")

    process.stdout.close()
    return_code = process.wait()
    full_output = "\n".join(output_lines)

    if check and return_code != 0:
        raise subprocess.CalledProcessError(return_code, command, output=full_output)

    return full_output


def run_smpmgr_with_retry_stream(
    ble_target: str,
    timeout: float,
    command_arguments: list,
    retries: int = DEFAULT_RETRIES,
    retry_delay: float = DEFAULT_RETRY_DELAY,
    progress_callback=None,
    check: bool = True,
) -> str:
    """Retry wrapper for streaming smpmgr output."""
    last_error = None
    for attempt in range(1, retries + 1):
        try:
            return run_smpmgr_stream(
                ble_target,
                timeout,
                command_arguments,
                progress_callback=progress_callback,
                check=check,
            )
        except subprocess.CalledProcessError as error:
            last_error = error
            if attempt < retries:
                time.sleep(retry_delay)
            else:
                raise

    if last_error:
        raise last_error
    raise RuntimeError("Unexpected retry loop exit")