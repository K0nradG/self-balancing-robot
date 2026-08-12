#!/usr/bin/env python3
"""
Cross-platform MCUboot DFU over Bluetooth LE using smpmgr.

On Windows it uses the patched smpmgr.exe from 3rdParty/smpmgr/.
On Linux it uses smpmgr installed via pipx (or available in PATH).
"""

import argparse
import re
import shutil
import subprocess
import sys
import time
import zipfile
from pathlib import Path

DEFAULT_BLE_TARGET = "SELF_BALANCING_ROBOT"
TARGET_FILENAME = "app.signed.bin"

BASE_DIR = Path(__file__).resolve().parent
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


class ProgressBar:
    """Simple progress bar for terminal output."""
    
    def __init__(self, total_steps=100, width=50, description=""):
        self.total_steps = total_steps
        self.current_step = 0
        self.width = width
        self.description = description
        self.start_time = time.monotonic()
        self._last_print_length = 0
        
    def update(self, step=None, description=None):
        """Update progress bar to specific step or increment by 1."""
        if step is not None:
            self.current_step = step
        else:
            self.current_step += 1
            
        if description:
            self.description = description
            
        # Ensure we don't exceed 100%
        self.current_step = min(self.current_step, self.total_steps)
        
        # Calculate percentage
        percentage = (self.current_step / self.total_steps) * 100
        
        # Calculate elapsed time
        elapsed = time.monotonic() - self.start_time
        
        # Build progress bar
        filled_length = int(self.width * self.current_step // self.total_steps)
        bar = '█' * filled_length + '░' * (self.width - filled_length)
        
        # Calculate ETA
        if self.current_step > 0 and self.current_step < self.total_steps:
            eta = (elapsed / self.current_step) * (self.total_steps - self.current_step)
            eta_str = f"ETA: {eta:.0f}s"
        else:
            eta_str = ""
        
        # Clear previous line and print new progress
        output = f'\r{self.description} [{bar}] {percentage:5.1f}% {eta_str}'
        
        # Pad with spaces to clear any remaining characters from previous output
        if len(output) < self._last_print_length:
            output += ' ' * (self._last_print_length - len(output))
        self._last_print_length = len(output)
        
        sys.stdout.write(output)
        sys.stdout.flush()
        
    def finish(self, message="\n"):
        """Complete the progress bar."""
        self.update(self.total_steps)
        sys.stdout.write(message)
        sys.stdout.flush()


def extract_firmware_from_zip() -> Path:
    if not ZIP_FILE_PATH.exists():
        print(f"Warning: Archive {ZIP_FILE_PATH} not found.")
        return DEFAULT_IMAGE

    print(f"Extracting {TARGET_FILENAME} from {ZIP_FILE_PATH} ...")
    try:
        with zipfile.ZipFile(ZIP_FILE_PATH, "r") as archive:
            if TARGET_FILENAME not in archive.namelist():
                print(
                    f"Error: {TARGET_FILENAME} not found in the root of "
                    f"{ZIP_FILE_PATH}"
                )
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
    print(f"Firmware extracted successfully: {DEFAULT_IMAGE}")
    print(f"Size: {file_size:,} bytes\n")
    return DEFAULT_IMAGE


def run_smpmgr(
    ble_target: str,
    timeout: int,
    *arguments: str,
    check: bool = True,
    capture_output: bool = True,
) -> subprocess.CompletedProcess[str]:
    executable = str(SMPMGR_PATH) if sys.platform == "win32" else SMPMGR_PATH
    command = [
        executable,
        "--timeout",
        str(timeout),
        "--ble",
        ble_target,
        *arguments,
    ]
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE if capture_output else None,
        stderr=subprocess.STDOUT if capture_output else None,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if check and result.returncode != 0:
        raise subprocess.CalledProcessError(result.returncode, command)
    return result


def run_smpmgr_with_retry(
    ble_target: str,
    timeout: int,
    *arguments: str,
    retries: int = DEFAULT_RETRIES,
    retry_delay: float = DEFAULT_RETRY_DELAY,
    check: bool = True,
    capture_output: bool = True,
) -> subprocess.CompletedProcess[str]:
    last_error = None
    for attempt in range(1, retries + 1):
        try:
            result = run_smpmgr(
                ble_target,
                timeout,
                *arguments,
                check=check,
                capture_output=capture_output,
            )
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


def extract_slot_hash(output: str, requested_slot: int = 1) -> str | None:
    blocks = re.findall(
        r"ImageState\s*\((.*?)(?=\s*ImageState\s*\(|\s*splitStatus\s*:|\Z)",
        output,
        flags=re.DOTALL | re.IGNORECASE,
    )

    for block in blocks:
        slot_match = re.search(
            r"\bslot\s*=\s*(\d+)",
            block,
            flags=re.IGNORECASE,
        )
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
    return None


def read_secondary_image_hash(
    ble_target: str,
    request_timeout: int,
    retries: int = DEFAULT_RETRIES,
    retry_delay: float = DEFAULT_RETRY_DELAY,
) -> str:

    for attempt in range(1, retries + 1):
        try:
            result = run_smpmgr_with_retry(
                ble_target,
                request_timeout,
                "image",
                "state-read",
                retries=1,
                capture_output=True,
            )
            output = result.stdout or ""

            image_hash = extract_slot_hash(output, requested_slot=1)
            if image_hash and len(image_hash) in (64, 96, 128):
                return image_hash

        except subprocess.CalledProcessError:
            pass

        if attempt < retries:
            time.sleep(retry_delay)

    raise RuntimeError("could not find a valid hash for MCUboot slot 1")


def format_duration(seconds: float) -> str:
    minutes, remaining_seconds = divmod(seconds, 60)
    if minutes:
        return f"{int(minutes)}m {remaining_seconds:.1f}s"
    return f"{remaining_seconds:.1f}s"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Perform a safe MCUboot test update over Bluetooth LE using "
            "smpmgr. Supports Windows and Linux."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  Linux:
    python3 dfu_update.py --ble "nRF54LM20B SMP" zephyr.signed.bin

  Windows PowerShell:
    py dfu_update.py --ble "nRF54LM20B SMP" zephyr.signed.bin

The default flow uploads, marks the secondary image for a test boot, resets,
reconnects, and confirms the running image. Use --no-confirm to keep it in
test mode so MCUboot can revert it after another reset.
""",
    )
    parser.add_argument(
        "image",
        nargs="?",
        type=Path,
        default=DEFAULT_IMAGE,
        help=f"signed MCUboot image (default: {DEFAULT_IMAGE})",
    )
    parser.add_argument(
        "--ble",
        default=DEFAULT_BLE_TARGET,
        metavar="NAME_OR_ADDRESS",
        help=f"BLE device name or address (default: {DEFAULT_BLE_TARGET})",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="leave the updated image in MCUboot test state",
    )
    parser.add_argument(
        "--request-timeout",
        type=int,
        default=DEFAULT_REQUEST_TIMEOUT,
        metavar="SECONDS",
        help=(
            "timeout for normal SMP requests "
            f"(default: {DEFAULT_REQUEST_TIMEOUT})"
        ),
    )
    parser.add_argument(
        "--upload-timeout",
        type=int,
        default=DEFAULT_UPLOAD_TIMEOUT,
        metavar="SECONDS",
        help=(
            "timeout for each upload request "
            f"(default: {DEFAULT_UPLOAD_TIMEOUT})"
        ),
    )
    parser.add_argument(
        "--reconnect-attempts",
        type=int,
        default=DEFAULT_RECONNECT_ATTEMPTS,
        metavar="COUNT",
        help=(
            "attempts to reconnect after reset "
            f"(default: {DEFAULT_RECONNECT_ATTEMPTS})"
        ),
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=DEFAULT_RECONNECT_DELAY,
        metavar="SECONDS",
        help=(
            "delay between reconnection attempts "
            f"(default: {DEFAULT_RECONNECT_DELAY:g})"
        ),
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_RETRIES,
        metavar="COUNT",
        help=(
            "number of retries for each SMP command "
            f"(default: {DEFAULT_RETRIES})"
        ),
    )
    parser.add_argument(
        "--retry-delay",
        type=float,
        default=DEFAULT_RETRY_DELAY,
        metavar="SECONDS",
        help=(
            "delay between retries of SMP command "
            f"(default: {DEFAULT_RETRY_DELAY:g})"
        ),
    )
    parser.add_argument(
        "--post-upload-delay",
        type=float,
        default=DEFAULT_POST_UPLOAD_DELAY,
        metavar="SECONDS",
        help=(
            "extra delay after upload before reading hash/state-write "
            f"(default: {DEFAULT_POST_UPLOAD_DELAY:g})"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if (
        args.request_timeout <= 0
        or args.upload_timeout <= 0
        or args.reconnect_attempts <= 0
        or args.reconnect_delay < 0
        or args.retries <= 0
        or args.retry_delay < 0
        or args.post_upload_delay < 0
    ):
        print(
            "Error: timeouts, retries, and delays must be positive; "
            "delays cannot be negative.",
            file=sys.stderr,
        )
        return 2

    if sys.platform == "win32":
        if not Path(SMPMGR_PATH).is_file():
            print(
                f"Error: smpmgr executable not found at {SMPMGR_PATH}\n"
                "Ensure it exists in 3rdParty/smpmgr/",
                file=sys.stderr,
            )
            return 1
    else:
        if shutil.which("smpmgr") is None:
            print(
                "Error: smpmgr is not installed or is not in PATH.\n"
                "Install it with: pipx install smpmgr",
                file=sys.stderr,
            )
            return 1

    default_image = extract_firmware_from_zip()
    image = args.image.expanduser().resolve()

    if str(image) == str(DEFAULT_IMAGE):
        image = default_image

    if not image.is_file():
        print(
            f"Error: signed firmware image not found: {image}\n"
            "Build the sample first, or pass the path to zephyr.signed.bin.",
            file=sys.stderr,
        )
        return 1

    print(f"BLE target: {args.ble}")
    print(f"Firmware:   {image}\n")

    dfu_started = time.monotonic()

    def finish(return_code: int, message: str = "") -> int:
        elapsed = time.monotonic() - dfu_started
        if message:
            print(f"\n{message}")
        print(f"Total DFU time: {format_duration(elapsed)}")
        return return_code

    # Create progress bar for overall process
    progress = ProgressBar(total_steps=100, width=50, description="DFU Progress")

    try:
        # Step 1: Check current state (0-5%)
        progress.update(0, "Checking current state...")
        run_smpmgr_with_retry(
            args.ble,
            args.request_timeout,
            "image",
            "state-read",
            retries=args.retries,
            retry_delay=args.retry_delay,
            capture_output=True,
        )
        progress.update(5)

        # Step 2: Upload image (5-60%)
        progress.update(5, "Uploading image...")
        upload_started = time.monotonic()
        run_smpmgr_with_retry(
            args.ble,
            args.upload_timeout,
            "image",
            "upload",
            "--format",
            "any",
            str(image),
            retries=args.retries,
            retry_delay=args.retry_delay,
            capture_output=True,
        )
        progress.update(60)
        
        if args.post_upload_delay > 0:
            time.sleep(args.post_upload_delay)

        # Step 3: Read hash (60-70%)
        progress.update(60, "Reading image hash...")
        image_hash = read_secondary_image_hash(
            args.ble,
            args.request_timeout,
            retries=args.retries,
            retry_delay=args.retry_delay,
        )
        progress.update(70)

        # Step 4: Write state (70-80%)
        progress.update(70, "Scheduling test boot...")
        run_smpmgr_with_retry(
            args.ble,
            args.request_timeout,
            "image",
            "state-write",
            image_hash,
            retries=args.retries,
            retry_delay=args.retry_delay,
            capture_output=True,
        )
        progress.update(80)

        # Step 5: Reset (80-85%)
        progress.update(80, "Resetting device...")
        run_smpmgr_with_retry(
            args.ble,
            args.request_timeout,
            "os",
            "reset",
            retries=args.retries,
            retry_delay=args.retry_delay,
            capture_output=True,
        )
        progress.update(85)

    except (subprocess.CalledProcessError, RuntimeError) as error:
        if isinstance(error, subprocess.CalledProcessError):
            detail = f"smpmgr exited with status {error.returncode}"
            return_code = error.returncode
        else:
            detail = str(error)
            return_code = 1
        return finish(return_code, f"Error: {detail}")

    if args.no_confirm:
        progress.update(100)
        progress.finish()
        print(
            "\nUpdate was left in MCUboot test mode.\n"
            "After verifying it, confirm with:\n"
            f'smpmgr --timeout {args.request_timeout} --ble "{args.ble}" '
            "image state-write --confirm"
        )
        return finish(0)

    # Step 6: Reconnect (85-95%)
    progress.update(85, "Waiting for device...")
    reconnect_success = False
    for attempt in range(1, args.reconnect_attempts + 1):
        result = run_smpmgr_with_retry(
            args.ble,
            args.request_timeout,
            "image",
            "state-read",
            retries=1,
            check=False,
            capture_output=True,
        )
        if result.returncode == 0:
            reconnect_success = True
            break
        if attempt < args.reconnect_attempts:
            time.sleep(args.reconnect_delay)

    if not reconnect_success:
        progress.finish()
        return finish(
            1,
            "Error: device did not become reachable after test boot. "
            "MCUboot rollback remains available."
        )

    progress.update(95, "Confirming image...")

    # Step 7: Confirm (95-100%)
    try:
        run_smpmgr_with_retry(
            args.ble,
            args.request_timeout,
            "image",
            "state-write",
            "--confirm",
            retries=args.retries,
            retry_delay=args.retry_delay,
            capture_output=True,
        )
    except subprocess.CalledProcessError as error:
        progress.finish()
        return finish(
            error.returncode,
            "Error: confirmation failed; MCUboot rollback remains available."
        )

    progress.update(100, "Update complete!")
    progress.finish()
    
    return finish(0, "DFU update completed successfully!")


if __name__ == "__main__":
    raise SystemExit(main())

