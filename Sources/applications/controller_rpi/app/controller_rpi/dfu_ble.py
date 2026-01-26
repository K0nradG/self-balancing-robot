import subprocess
import sys
import os
import time
from time import perf_counter
import re

NEWTMGR = "sudo ~/mynewt-newtmgr/newtmgr/newtmgr"


def run(cmd, check=True, retries=1, delay=1, print_cmd=True):
    for attempt in range(1, retries + 1):
        if print_cmd:
            print(f">>> {cmd} (attempt {attempt}/{retries})", flush=True)
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout, end="" if result.stdout.endswith("\n") else "\n", flush=True)
        if result.returncode == 0:
            return result
        if attempt < retries:
            print(f"Command failed (rc={result.returncode}), retrying in {delay}s...", flush=True)
            time.sleep(delay)
    if check:
        if result.stderr:
            print(result.stderr, flush=True)
        sys.exit(result.returncode)
    return result


def enable_bluetooth():
    print(">>> Enabling Bluetooth and hci0...", flush=True)
    run("sudo rfkill unblock bluetooth", check=False)
    run("sudo hciconfig hci0 up", check=False)


def extract_slot1_hash(output: str):
    lines = output.splitlines()
    slot1 = False
    for line in lines:
        if "slot=1" in line:
            slot1 = True
        elif slot1 and "hash:" in line:
            return line.split("hash:")[1].strip()
        elif "slot=" in line:
            slot1 = False
    return None


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <IMAGE_PATH>", flush=True)
        sys.exit(1)

    image_path = sys.argv[1]
    if not os.path.isfile(image_path):
        print(f"File does not exist: {image_path}", flush=True)
        sys.exit(1)

    file_size_bytes = os.path.getsize(image_path)
    file_size_kib = file_size_bytes / 1024.0
    program_start = perf_counter()

    enable_bluetooth()
    run(f'{NEWTMGR} conn delete bleconn', check=False)
    run(f'{NEWTMGR} conn add bleconn type=ble connstring="peer_name=SELF_BALANCING_ROBOT"', retries=3, delay=2)
    run(f"{NEWTMGR} image list -c bleconn", retries=5, delay=2)

    # --- DFU Upload ---
    print("[DFU_STEP] Uploading image to the device", flush=True)
    upload_start = perf_counter()
    process = subprocess.Popen(
        f"{NEWTMGR} image upload -c bleconn {image_path}",
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    for line in process.stdout:
        line = line.strip()
        if not line:
            continue
        print(line, flush=True)

        # Parsowanie procentów i prędkości
        m = re.search(r"\s([0-9.]+)%\s+([0-9.]+)\s*KiB/s", line)
        if m:
            percent = float(m.group(1))
            speed = float(m.group(2))
            print(f"[DFU_PROGRESS] {percent:.1f}% | {speed:.1f} KiB/s", flush=True)

    process.wait()
    upload_end = perf_counter()
    upload_elapsed = upload_end - upload_start
    avg_speed = file_size_kib / upload_elapsed if upload_elapsed > 0 else 0.0

    # --- Verifying image ---
    print("[DFU_STEP] Verifying image", flush=True)
    image_hash = None
    for attempt in range(1, 10):
        res = run(f"{NEWTMGR} image list -c bleconn", check=False)
        image_hash = extract_slot1_hash(res.stdout)
        if image_hash:
            break
        print(f"[DFU_STEP] Verifying image... attempt {attempt}/10", flush=True)
        time.sleep(2)
    if not image_hash:
        print("Failed to find new image hash", flush=True)
        sys.exit(1)
    print(f">>> Found slot1 hash: {image_hash}", flush=True)

    # --- Testing image ---
    print("[DFU_STEP] Testing image", flush=True)
    run(f"{NEWTMGR} image test -c bleconn {image_hash}")
    print("[DFU_STEP] Waiting for device to reboot", flush=True)

    run(f"{NEWTMGR} reset -c bleconn", check=False)
    time.sleep(10)
    run(f"{NEWTMGR} image list -c bleconn", check=False)

    total_elapsed = perf_counter() - program_start
    print(f"[DFU_STATS] time={total_elapsed:.1f}s avg_speed={avg_speed:.1f}KiB/s", flush=True)
    print("[DFU_STEP] DFU finished successfully", flush=True)


if __name__ == "__main__":
    main()
