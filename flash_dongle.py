import subprocess
import time
import re
import sys
import os


# ============================================================
# Configuration
# ============================================================

SMPMGR_PATH = r".\smpmgr.exe"
DEVICE_NAME = "SELF_BALANCING_ROBOT"
FIRMWARE_PATH = r".\app.signed.bin"

TIMEOUT = "10"
MAX_RETRIES = 4
RETRY_DELAY = 3


# ============================================================
# Run smpmgr
# ============================================================

def run_cmd(args, retries=MAX_RETRIES, capture=False):
    """
    Run smpmgr.

    capture=False:
        smpmgr output goes directly to the console.

    capture=True:
        stdout/stderr are captured and returned as a string.
    """

    command = [
        SMPMGR_PATH,
        "--timeout",
        TIMEOUT,
        "--ble",
        DEVICE_NAME,
    ] + args

    cmd_str = " ".join(command)

    env = os.environ.copy()

    # Force UTF-8 for Python processes.
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"

    if capture:
        # Keep captured output simple.
        env["TERM"] = "dumb"
        env["NO_COLOR"] = "1"

    for attempt in range(1, retries + 1):

        print()
        print(
            f"[Attempt {attempt}/{retries}] "
            f"Executing: {cmd_str}"
        )
        print()

        if capture:

            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                env=env,
            )

            stdout = result.stdout or ""
            stderr = result.stderr or ""

            output = stdout + "\n" + stderr

            print(output)

        else:

            # Let smpmgr write directly to the console.
            result = subprocess.run(
                command,
                env=env,
            )

            output = ""

        if result.returncode == 0:
            time.sleep(2)
            return output

        print()
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


# ============================================================
# Extract slot hash
# ============================================================

def extract_slot_hash(output, requested_slot=1):
    """
    Extract the hash belonging specifically to ImageState
    slot=N.

    Example smpmgr output:

        ImageState(
          slot=1,
          version='2.0.0',
          image=None,
          hash=HashBytes(
          '8B6C242ED7A2F4777849F8C607C9109EA07F101B0CB6481E209F02A33A16FD85'
          ),
          bootable=True,
          pending=False,
          confirmed=False,
          active=False,
          permanent=False
        )

    The important detail is that HashBytes(...) itself contains
    parentheses, so we must NOT use the first ")" as the end
    of ImageState.
    """

    print()
    print("============================================================")
    print(
        f"Searching for hash belonging to slot {requested_slot}"
    )
    print("============================================================")

    # --------------------------------------------------------
    # Find ImageState blocks.
    #
    # We cannot simply search until ")" because HashBytes(...)
    # has its own closing ")".
    #
    # Instead, each ImageState block ends immediately before
    # the next ImageState(...) or splitStatus line.
    # --------------------------------------------------------

    blocks = re.findall(
        r"ImageState\s*\((.*?)(?=\n\s*ImageState\s*\(|\n\s*splitStatus\s*:|\Z)",
        output,
        flags=re.DOTALL | re.IGNORECASE,
    )

    print(f"Found {len(blocks)} ImageState block(s).")

    for index, block in enumerate(blocks, start=1):

        # ----------------------------------------------------
        # Extract slot number from this ImageState block.
        # ----------------------------------------------------

        slot_match = re.search(
            r"\bslot\s*=\s*(\d+)",
            block,
            flags=re.IGNORECASE,
        )

        if not slot_match:
            print(
                f"ImageState #{index}: no slot found"
            )
            continue

        slot = int(slot_match.group(1))

        print(
            f"ImageState #{index}: slot={slot}"
        )

        # Not the slot we are looking for.
        if slot != requested_slot:
            continue

        # ----------------------------------------------------
        # We found the requested slot.
        #
        # Now find HashBytes('...')
        # ----------------------------------------------------

        hash_match = re.search(
            r"hash\s*=\s*HashBytes\s*\(\s*['\"]([0-9A-Fa-f]+)['\"]\s*\)",
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

        print(
            f"Slot {requested_slot} was found, "
            "but no HashBytes value was found in that block."
        )

    return None


# ============================================================
# Main
# ============================================================

def main():

    # --------------------------------------------------------
    # STEP 1: Upload firmware
    # --------------------------------------------------------

    print("============================================================")
    print(" STEP 1: Uploading Firmware")
    print("============================================================")

    run_cmd(
        ["image", "upload", FIRMWARE_PATH],
        capture=False,
    )

    print()
    print("Upload completed successfully.")

    # --------------------------------------------------------
    # STEP 2: Read image states
    # --------------------------------------------------------

    print()
    print("============================================================")
    print(" STEP 2: Reading Image States")
    print("============================================================")

    state_output = run_cmd(
        ["image", "state-read"],
        capture=True,
    )

    # --------------------------------------------------------
    # Extract slot 1 hash
    # --------------------------------------------------------

    firmware_hash = extract_slot_hash(
        state_output,
        requested_slot=1,
    )

    if not firmware_hash:

        print()
        print(
            "ERROR: Could not find hash for slot 1 "
            "in the device response."
        )
        print()

        sys.exit(1)

    print()
    print(
        f"-> Extracted Slot 1 Hash: "
        f"{firmware_hash}"
    )

    # --------------------------------------------------------
    # STEP 3: Mark image for test
    # --------------------------------------------------------

    print()
    print("============================================================")
    print(" STEP 3: Marking Image for Test")
    print("============================================================")

    run_cmd(
        ["image", "state-write", firmware_hash],
        capture=False,
    )

    print()
    print("Image marked for test successfully.")

    # --------------------------------------------------------
    # STEP 4: Reset device
    # --------------------------------------------------------

    print()
    print("============================================================")
    print(" STEP 4: Resetting Device")
    print("============================================================")

    run_cmd(
        ["os", "reset"],
        capture=False,
    )

    wait_time = 15

    print()
    print(
        f"Waiting {wait_time} seconds "
        "for device to reboot..."
    )

    time.sleep(wait_time)

    # --------------------------------------------------------
    # STEP 5: Confirm image
    # --------------------------------------------------------

    print()
    print("============================================================")
    print(" STEP 5: Confirming Image")
    print("============================================================")

    run_cmd(
        ["image", "state-write", "--confirm"],
        capture=False,
    )

    # --------------------------------------------------------
    # DONE
    # --------------------------------------------------------

    print()
    print("============================================================")
    print(" OTA UPDATE COMPLETE")
    print("============================================================")
    print()
    print(
        f"Slot 1 hash: {firmware_hash}"
    )
    print()


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":
    main()

