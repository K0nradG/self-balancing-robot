import subprocess
import sys
import os
import time

NEWTMGR = "sudo /usr/local/bin/newtmgr"

def run(cmd, check=True):
    print(f">>> {cmd}")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    print(result.stdout)
    if check and result.returncode != 0:
        print(result.stderr)
        sys.exit(result.returncode)
    return result

def bluetoothctl_cmds(addr):
    cmds = f"""
    power on
    agent on
    default-agent
    trust {addr}
    pair {addr}
    connect {addr}
    quit
    """
    print(">>> Connecting via bluetoothctl...")
    p = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = p.communicate(cmds)
    print(out)
    if p.returncode != 0:
        print(err)
        sys.exit(p.returncode)
    time.sleep(2)  # Wait a moment for the connection to stabilize

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <BLE_ADDRESS> <IMAGE_PATH>")
        sys.exit(1)

    ble_addr = sys.argv[1]
    image_path = sys.argv[2]

    if not os.path.isfile(image_path):
        print(f"File does not exist: {image_path}")
        sys.exit(1)

    bluetoothctl_cmds(ble_addr)

    # Add BLE connection profile for newtmgr
    run(f"{NEWTMGR} conn add bleconn type=ble connstring=\"peer_name=AliroDL\"")
    # List images on the device
    run(f"{NEWTMGR} image list -c bleconn")
    # Upload new image
    run(f"{NEWTMGR} image upload -c bleconn {image_path}")

    print(">>> Fetching hash of the new image...")
    result = run(f"{NEWTMGR} image list -c bleconn", check=False)
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

    # Set the new image as pending (test)
    run(f"{NEWTMGR} image test -c bleconn {image_hash}")
    # Reset the device
    run(f"{NEWTMGR} reset -c bleconn")

if __name__ == "__main__":
    main()
