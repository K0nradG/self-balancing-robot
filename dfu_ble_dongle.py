#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

from dfu_utils.config import (
    DEFAULT_BLE_TARGET, DEFAULT_IMAGE, DEFAULT_POST_UPLOAD_DELAY,
    DEFAULT_RECONNECT_ATTEMPTS, DEFAULT_RECONNECT_DELAY, DEFAULT_REQUEST_TIMEOUT,
    DEFAULT_RETRIES, DEFAULT_RETRY_DELAY, DEFAULT_UPLOAD_TIMEOUT
)
from dfu_utils.dfu_operations import DFUOperations
from dfu_utils.firmware_utils import extract_firmware_from_zip
from dfu_utils.smpmgr_wrapper import check_smpmgr_available


def parse_args():
    parser = argparse.ArgumentParser(description="MCUboot DFU over Bluetooth LE using smpmgr")
    parser.add_argument("image", nargs="?", type=Path, default=DEFAULT_IMAGE,
                       help=f"signed MCUboot image (default: {DEFAULT_IMAGE})")
    parser.add_argument("--ble", default=DEFAULT_BLE_TARGET, metavar="NAME_OR_ADDRESS",
                       help=f"BLE device name or address (default: {DEFAULT_BLE_TARGET})")
    parser.add_argument("--no-confirm", action="store_true",
                       help="leave the updated image in MCUboot test state")
    parser.add_argument("--request-timeout", type=int, default=DEFAULT_REQUEST_TIMEOUT, metavar="SECONDS")
    parser.add_argument("--upload-timeout", type=int, default=DEFAULT_UPLOAD_TIMEOUT, metavar="SECONDS")
    parser.add_argument("--reconnect-attempts", type=int, default=DEFAULT_RECONNECT_ATTEMPTS, metavar="COUNT")
    parser.add_argument("--reconnect-delay", type=float, default=DEFAULT_RECONNECT_DELAY, metavar="SECONDS")
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES, metavar="COUNT")
    parser.add_argument("--retry-delay", type=float, default=DEFAULT_RETRY_DELAY, metavar="SECONDS")
    parser.add_argument("--post-upload-delay", type=float, default=DEFAULT_POST_UPLOAD_DELAY, metavar="SECONDS")
    return parser.parse_args()


def validate_args(args):
    if (args.request_timeout <= 0 or args.upload_timeout <= 0 or 
        args.reconnect_attempts <= 0 or args.reconnect_delay < 0 or
        args.retries <= 0 or args.retry_delay < 0 or args.post_upload_delay < 0):
        print("Error: Invalid timeout/retry values", file=sys.stderr)
        return False
    return True


def main():
    args = parse_args()
    
    if not validate_args(args):
        return 2
    
    is_available, error_message = check_smpmgr_available()
    if not is_available:
        print(f"Error: {error_message}", file=sys.stderr)
        return 1
    
    default_image = extract_firmware_from_zip()
    image = args.image.expanduser().resolve()
    
    if str(image) == str(DEFAULT_IMAGE):
        image = default_image
    
    if not image.is_file():
        print(f"Error: signed firmware image not found: {image}", file=sys.stderr)
        return 1
    
    print(f"BLE target: {args.ble}")
    print(f"Firmware:   {image}")
    print(f"Image size: {image.stat().st_size:,} bytes\n")
    
    dfu = DFUOperations(
        ble_target=args.ble,
        image_path=image,
        request_timeout=args.request_timeout,
        upload_timeout=args.upload_timeout,
        reconnect_attempts=args.reconnect_attempts,
        reconnect_delay=args.reconnect_delay,
        retries=args.retries,
        retry_delay=args.retry_delay,
        post_upload_delay=args.post_upload_delay,
        no_confirm=args.no_confirm,
    )
    
    return_code, message = dfu.perform_update()
    print(f"\n{message}")
    
    if args.no_confirm and return_code == 0:
        print(f'smpmgr --timeout {args.request_timeout} --ble "{args.ble}" image state-write --confirm')
    
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())

