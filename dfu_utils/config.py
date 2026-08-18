#!/usr/bin/env python3

import sys
from pathlib import Path

DEFAULT_BLE_TARGET = "SELF_BALANCING_ROBOT"
TARGET_FILENAME = "app.signed.bin"

BASE_DIR = Path(__file__).resolve().parent.parent
BUILD_DIR = BASE_DIR / "Sources" / "applications" / "app" / "build"
ZIP_FILE_PATH = BUILD_DIR / "dfu_application.zip"
EXTRACT_DIR = BUILD_DIR / "extracted"
EXTRACT_DIR.mkdir(parents=True, exist_ok=True)

DEFAULT_IMAGE = EXTRACT_DIR / TARGET_FILENAME

DEFAULT_REQUEST_TIMEOUT = 10
DEFAULT_UPLOAD_TIMEOUT = 40
DEFAULT_RECONNECT_ATTEMPTS = 12
DEFAULT_RECONNECT_DELAY = 5.0
DEFAULT_RETRIES = 5
DEFAULT_RETRY_DELAY = 2.0
DEFAULT_POST_UPLOAD_DELAY = 3.0 if sys.platform == "win32" else 0.0

if sys.platform == "win32":
    SMPMGR_PATH = BASE_DIR / "3rdParty" / "smpmgr" / "smpmgr.exe"
else:
    SMPMGR_PATH = "smpmgr"

PROGRESS_STEPS = {
    "start": 0,
    "state_read": 5,
    "upload": 60,
    "hash_read": 70,
    "state_write": 80,
    "reset": 85,
    "reconnect": 95,
    "confirm": 100,
}
