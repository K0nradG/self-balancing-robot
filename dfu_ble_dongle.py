import subprocess
import time
import re
import sys
import os
import zipfile
import shutil

DEVICE_NAME = "SELF_BALANCING_ROBOT"
SMPMGR_PATH = r".\3rdParty\smpmgr\smpmgr.exe"
DFU_ZIP_PATH = r".\Sources\applications\app\build\dfu_application.zip"

FIRMWARE_PATH = os.path.join(
    os.path.dirname(DFU_ZIP_PATH),
    "app.signed.bin",
)

TIMEOUT = "10"
MAX_RETRIES = 5
RETRY_DELAY = 2
REBOOT_WAIT = 15


def extract_firmware_from_zip():

    zip_path = os.path.abspath(DFU_ZIP_PATH)
    firmware_path = os.path.abspath(FIRMWARE_PATH)

    if not os.path.isfile(zip_path):
        print("ERROR: Firmware ZIP was not found:")
        print(f"  {zip_path}")
        sys.exit(1)

    try:
        with zipfile.ZipFile(zip_path, "r") as archive:

            if "app.signed.bin" not in archive.namelist():
                print()
                print(
                    "ERROR: app.signed.bin was not found "
                    "in the root of dfu_application.zip."
                )
                sys.exit(1)

            with archive.open("app.signed.bin", "r") as source, open(
                firmware_path,
                "wb",
            ) as destination:
                shutil.copyfileobj(source, destination)

    except zipfile.BadZipFile:
        print()
        print(
            "ERROR: dfu_application.zip is invalid "
            "or corrupted."
        )
        print(f"  {zip_path}")
        sys.exit(1)

    except Exception as exc:
        print()
        print("ERROR: Failed to extract firmware:")
        print(f"  {exc}")
        sys.exit(1)

    if not os.path.isfile(firmware_path):
        print()
        print("ERROR: app.signed.bin was not created.")
        sys.exit(1)

    file_size = os.path.getsize(firmware_path)

    print()
    print("Firmware extracted successfully.")
    print(f"  File: {firmware_path}")
    print(f"  Size: {file_size:,} bytes")
    print()


def run_cmd(args, retries=MAX_RETRIES):

    command = [
        SMPMGR_PATH,
        "--timeout",
        TIMEOUT,
        "--ble",
        DEVICE_NAME,
    ] + args

    cmd_str = " ".join(command)

    for attempt in range(1, retries + 1):

        print(
            f"[Attempt {attempt}/{retries}] "
            f"Executing: {cmd_str}"
        )

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        output = (
            (result.stdout or "")
            + "\n"
            + (result.stderr or "")
        )

        print(output)

        if result.returncode == 0:
            return output

        print(
            f"Command failed with return code "
            f"{result.returncode}."
        )

        if attempt < retries:

            print(
                f"Retrying in {RETRY_DELAY} seconds..."
            )

            time.sleep(RETRY_DELAY)

        else:

            print()
            print("Max retries reached. Exiting.")
            sys.exit(1)

    return ""


def extract_slot_hash(output, requested_slot=1):

    blocks = re.findall(
        r"ImageState\s*\((.*?)(?=\s*ImageState\s*\(|\s*splitStatus\s*:|\Z)",
        output,
        flags=re.DOTALL | re.IGNORECASE,
    )

    print(
        f"Found {len(blocks)} ImageState block(s)."
    )

    for index, block in enumerate(
        blocks,
        start=1,
    ):

        slot_match = re.search(
            r"\bslot\s*=\s*(\d+)",
            block,
            flags=re.IGNORECASE,
        )

        if not slot_match:
            continue

        slot = int(
            slot_match.group(1)
        )

        print(
            f"ImageState #{index}: slot={slot}"
        )

        if slot != requested_slot:
            continue

        hash_match = re.search(
            r"\bhash\s*=\s*HashBytes\s*\(\s*['\"]([0-9A-Fa-f]+)['\"]\s*\)",
            block,
            flags=re.DOTALL | re.IGNORECASE,
        )

        if hash_match:

            firmware_hash = hash_match.group(1).upper()

            print()
            print(
                f"Found slot {requested_slot} hash:"
            )
            print(
                f"  {firmware_hash}"
            )
            print()

            return firmware_hash

    return None


def main():

    start_time = time.perf_counter()

    print("============================================================")
    print(" STEP 1: Extracting Firmware")
    print("============================================================")

    extract_firmware_from_zip()

    print()
    print("============================================================")
    print(" STEP 2: Uploading Firmware")
    print("============================================================")
    print()

    print(
        f"Firmware: {os.path.abspath(FIRMWARE_PATH)}"
    )

    run_cmd(
        [
            "image",
            "upload",
            FIRMWARE_PATH,
        ]
    )

    print()
    print("Upload completed successfully.")

    print()
    print("============================================================")
    print(" STEP 3: Reading Image States")
    print("============================================================")

    state_output = run_cmd(
        [
            "image",
            "state-read",
        ]
    )

    firmware_hash = extract_slot_hash(
        state_output,
        requested_slot=1,
    )

    if firmware_hash is None:

        print(
            "ERROR: Could not find hash for slot 1 "
            "in the device response."
        )

        sys.exit(1)

    print()
    print(
        f"-> Extracted Slot 1 Hash: "
        f"{firmware_hash}"
    )

    print()
    print("============================================================")
    print(" STEP 4: Marking Image for Test")
    print("============================================================")

    run_cmd(
        [
            "image",
            "state-write",
            firmware_hash,
        ]
    )

    print()
    print("Image marked for test successfully.")

    print()
    print("============================================================")
    print(" STEP 5: Resetting Device")
    print("============================================================")

    run_cmd(
        [
            "os",
            "reset",
        ]
    )

    print()
    print(
        f"Waiting {REBOOT_WAIT} seconds "
        "for device to reboot..."
    )

    time.sleep(REBOOT_WAIT)

    print()
    print("============================================================")
    print(" STEP 6: Confirming Image")
    print("============================================================")

    run_cmd(
        [
            "image",
            "state-write",
            "--confirm",
        ]
    )

    elapsed_seconds = time.perf_counter() - start_time

    hours = int(elapsed_seconds // 3600)
    minutes = int((elapsed_seconds % 3600) // 60)
    seconds = elapsed_seconds % 60

    print()
    print("============================================================")
    print(" OTA UPDATE COMPLETE")
    print("============================================================")
    print()

    print(f"Slot 1 hash: {firmware_hash}")
    print(
        f"Firmware file: "
        f"{os.path.abspath(FIRMWARE_PATH)}"
    )

    print()
    print("============================================================")
    print(" TOTAL EXECUTION TIME")
    print("============================================================")
    print()

    print(
        f"{hours:02d}:{minutes:02d}:{seconds:05.2f}"
    )

    print(
        f"Total: {elapsed_seconds:.2f} seconds"
    )

    print()


if __name__ == "__main__":
    main()
