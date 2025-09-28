import subprocess
import sys
import os
import time

NEWTMGR = "sudo /usr/local/bin/newtmgr"

def run(cmd, check=True, retries=1, delay=1):
    for attempt in range(1, retries + 1):
        print(f">>> {cmd} (attempt {attempt}/{retries})")
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        print(result.stdout)
        if result.returncode == 0:
            return result
        if attempt < retries:
            print(f"Command failed (rc={result.returncode}), retrying in {delay}s...")
            time.sleep(delay)
    if check:
        print(result.stderr)
        sys.exit(result.returncode)
    return result

def enable_bluetooth():
    print(">>> Odblokowywanie Bluetooth i uruchamianie hci0...")
    run("sudo rfkill unblock bluetooth", check=False)
    run("sudo hciconfig hci0 up", check=False)

def main():
    enable_bluetooth()

    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <BLE_ADDRESS> <IMAGE_PATH>")
        sys.exit(1)

    ble_addr = sys.argv[1]
    image_path = sys.argv[2]

    if not os.path.isfile(image_path):
        print(f"File does not exist: {image_path}")
        sys.exit(1)

    run(f'{NEWTMGR} conn delete bleconn', check=False)
    run(f'{NEWTMGR} conn add bleconn type=ble connstring="peer_name=SELF_BALANCING_ROBOT"', retries=3, delay=2)

    run(f"{NEWTMGR} image list -c bleconn", retries=5, delay=2)
    run(f"{NEWTMGR} image upload -c bleconn {image_path}", retries=3, delay=2)

    result = run(f"{NEWTMGR} image list -c bleconn", check=False, retries=5, delay=2)
    lines = result.stdout.splitlines()
    slot1 = False
    image_hash = None
    for line in lines:
        if "slot=1" in line:
            slot1 = True
        elif slot1 and "hash:" in line:
            image_hash = line.split("hash:")[1].strip()
            break
        elif "slot=" in line:
            slot1 = False
    if not image_hash:
        print("Could not find hash of the new image in slot 1!")
        sys.exit(1)

    run(f"{NEWTMGR} image test -c bleconn {image_hash}")
    run(f"{NEWTMGR} reset -c bleconn")
    print(">>> Waiting for device to reboot...")
    time.sleep(3)
    run(f"{NEWTMGR} image list -c bleconn", retries=5, delay=2)

if __name__ == "__main__":
    main()

