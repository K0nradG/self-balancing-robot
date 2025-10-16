import subprocess
import sys
import os
import time
from time import perf_counter

NEWTMGR = "sudo /usr/local/bin/newtmgr"

def run(cmd, check=True, retries=1, delay=1, print_cmd=True):
    """Uruchamia polecenie w shellu z obsługą retry i ewentualnym opóźnieniem."""
    for attempt in range(1, retries + 1):
        if print_cmd:
            print(f">>> {cmd} (attempt {attempt}/{retries})")
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
        if result.returncode == 0:
            return result
        if attempt < retries:
            print(f"Command failed (rc={result.returncode}), retrying in {delay}s...")
            time.sleep(delay)
    if check:
        if result.stderr:
            print(result.stderr)
        sys.exit(result.returncode)
    return result

def enable_bluetooth():
    print(">>> Odblokowywanie Bluetooth i uruchamianie hci0...")
    run("sudo rfkill unblock bluetooth", check=False)
    run("sudo hciconfig hci0 up", check=False)

def extract_slot1_hash(image_list_output: str):
    """Wyciąga hash obrazu ze slotu 1 z wyjścia 'newtmgr image list'."""
    lines = image_list_output.splitlines()
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
        print(f"Usage: {sys.argv[0]} <IMAGE_PATH>")
        sys.exit(1)

    program_start = perf_counter()

    image_path = sys.argv[1]

    if not os.path.isfile(image_path):
        print(f"File does not exist: {image_path}")
        sys.exit(1)

    file_size_bytes = os.path.getsize(image_path)
    file_size_kib = file_size_bytes / 1024.0

    enable_bluetooth()

    # Przygotowanie połączenia BLE
    run(f'{NEWTMGR} conn delete bleconn', check=False)
    run(f'{NEWTMGR} conn add bleconn type=ble connstring="peer_name=SELF_BALANCING_ROBOT"', retries=3, delay=2)

    run(f"{NEWTMGR} image list -c bleconn", retries=5, delay=2)

    # --- Pomiar czasu uploadu DFU ---
    print(">>> Start DFU upload...")
    upload_start = perf_counter()
    run(f"{NEWTMGR} image upload -c bleconn {image_path}", retries=3, delay=0)
    upload_end = perf_counter()

    upload_elapsed = upload_end - upload_start
    avg_speed_kib = file_size_kib / upload_elapsed if upload_elapsed > 0 else 0.0

    print(f">>> DFU upload done. Czas: {upload_elapsed:.2f} s ({upload_elapsed/60:.2f} min), "
          f"Rozmiar: {file_size_kib:.2f} KiB, Śr. prędkość: {avg_speed_kib:.2f} KiB/s")

    # --- Pobranie hash nowego obrazu ---
    print(">>> Fetching hash of the new image (slot 1)...")
    image_hash = None
    max_attempts = 10
    for attempt in range(1, max_attempts + 1):
        result = run(f"{NEWTMGR} image list -c bleconn", check=False)
        image_hash = extract_slot1_hash(result.stdout)
        if image_hash:
            break
        print(f"Hash not found, waiting... ({attempt}/{max_attempts})")
        time.sleep(3)

    if not image_hash:
        print("Could not find hash of the new image in slot 1!")
        sys.exit(1)

    print(f">>> Found slot1 hash: {image_hash}")
    run(f"{NEWTMGR} image test -c bleconn {image_hash}")
    run(f"{NEWTMGR} reset -c bleconn")

    print(">>> Waiting for device to reboot...")
    time.sleep(3)
    run(f"{NEWTMGR} image list -c bleconn", retries=5, delay=2)

    # --- Całkowity czas ---
    total_elapsed = perf_counter() - program_start
    print(f">>> Total script time: {total_elapsed:.2f} s ({total_elapsed/60:.2f} min)")

if __name__ == "__main__":
    main()