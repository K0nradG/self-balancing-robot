#!/usr/bin/env python3

import re
import shutil
import sys
import time
import zipfile

from .config import (
    DEFAULT_IMAGE,
    DEFAULT_RETRIES,
    DEFAULT_RETRY_DELAY,
    TARGET_FILENAME,
    ZIP_FILE_PATH,
)
from .smpmgr_wrapper import run_smpmgr


def extract_firmware_from_zip():

    if not ZIP_FILE_PATH.exists():
        print(f"Warning: Archive {ZIP_FILE_PATH} not found.")
        return DEFAULT_IMAGE

    print(f"Extracting {TARGET_FILENAME} from {ZIP_FILE_PATH} ...")

    try:
        with zipfile.ZipFile(ZIP_FILE_PATH, "r") as archive:
            if TARGET_FILENAME not in archive.namelist():
                print(f"Error: {TARGET_FILENAME} not found in {ZIP_FILE_PATH}")
                sys.exit(1)

            with archive.open(TARGET_FILENAME, "r") as source, open(
                DEFAULT_IMAGE, "wb"
            ) as destination:
                shutil.copyfileobj(source, destination)

    except zipfile.BadZipFile:
        print(f"Error: {ZIP_FILE_PATH} is invalid or corrupted.")
        sys.exit(1)
    except Exception as exc:
        print(f"Error: Failed to extract firmware: {exc}")
        sys.exit(1)

    if not DEFAULT_IMAGE.is_file():
        print(f"Error: {TARGET_FILENAME} was not created.")
        sys.exit(1)

    file_size = DEFAULT_IMAGE.stat().st_size
    print(f"Firmware extracted: {DEFAULT_IMAGE} ({file_size:,} bytes)\n")

    return DEFAULT_IMAGE


def extract_slot_hash(output, requested_slot=1) -> str:

    blocks = re.findall(
        r"ImageState\s*\((.*?)(?=\s*ImageState\s*\(|\s*splitStatus\s*:|\Z)",
        output,
        flags=re.DOTALL | re.IGNORECASE,
    )

    for block in blocks:
        slot_match = re.search(r"\bslot\s*=\s*(\d+)", block, flags=re.IGNORECASE)
        if not slot_match:
            continue

        slot = int(slot_match.group(1))
        if slot != requested_slot:
            continue

        hash_match = re.search(
            r"\bhash\s*=\s*HashBytes\s*\(\s*['\"]([0-9A-Fa-f]+)['\"]\s*\)",
            block,
            flags=re.DOTALL | re.IGNORECASE,
        )
        if hash_match:
            return hash_match.group(1).upper()

    return ""


def read_secondary_image_hash(
    ble_target,
    request_timeout,
    retries=DEFAULT_RETRIES,
    retry_delay=DEFAULT_RETRY_DELAY,
):

    command_arguments = ["image", "state-read"]
    for attempt in range(1, retries + 1):
        try:
            result = run_smpmgr(
                ble_target,
                request_timeout,
                command_arguments,
                check=True,
                capture_output=True,
            )
            output = result.stdout or ""
            image_hash = extract_slot_hash(output, requested_slot=1)
            if len(image_hash) in (64, 96, 128):
                return image_hash

        except Exception:
            pass

        if attempt < retries:
            time.sleep(retry_delay)

    raise RuntimeError("could not find a valid hash for MCUboot slot 1")


def format_duration(seconds):

    minutes, remaining_seconds = divmod(seconds, 60)
    if minutes:
        return f"{int(minutes)}m {remaining_seconds:.1f}s"
    return f"{remaining_seconds:.1f}s"
