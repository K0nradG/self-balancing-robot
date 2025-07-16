import subprocess
import os
import re
from time import sleep


#if there are any changes to Dockerfile you have to build docker image first
#docker build -t nrfutil-python310 .

def check_docker_image():
    """Check if the Docker image 'nrfutil-python310' exists"""
    try:
        subprocess.run(
            ["docker", "inspect", "nrfutil-python310"],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return True
    except:
        return False

def find_nrf_device():
    """Find connected nRF device and return the serial port path"""
    try:
        # Check USB devices
        lsusb = subprocess.run(["lsusb"], capture_output=True, text=True)
        if "Nordic Semiconductor" not in lsusb.stdout:
            print("Nordic Semiconductor device not found.")
            return None

        # Look for ttyACM ports (commonly used by nRF devices)
        ports = []
        for dev in os.listdir('/dev'):
            if dev.startswith('ttyACM'):
                ports.append(f"/dev/{dev}")

        if not ports:
            print("No serial port found for nRF device.")
            return None

        # Return the first available port (extend this logic if needed)
        return ports[0]

    except Exception as e:
        print(f"Error while searching for device: {e}")
        return None

def run_in_docker(cmd, usb=False):
    """Run a command inside the Docker container, optionally with USB access"""
    if not check_docker_image():
        print("Docker image not found. Building the image...")
        subprocess.run(["docker", "build", "-t", "nrfutil-python310", "."], check=True)
    
    docker_cmd = [
        "docker", "run", "--rm",
        "-v", f"{os.getcwd()}:/app",
        "-w", "/app"
    ]
    
    if usb:
        docker_cmd.extend(["--privileged", "-v", "/dev:/dev"])
        
    docker_cmd.extend(["nrfutil-python310"] + cmd)
    subprocess.run(docker_cmd, check=True)

def flash_with_docker():
    """Generate DFU package and flash the firmware using nrfutil inside Docker"""
    hex_path = "build/app/zephyr/zephyr.hex"
    if not os.path.exists(hex_path):
        print("Error: Firmware not found. Please build it first using 'west build'.")
        return

    print("Generating DFU package...")
    run_in_docker([
        "nrfutil", "pkg", "generate",
        "--hw-version", "52",
        "--sd-req", "0x00",
        "--application", f"/app/{hex_path}",
        "--application-version", "1",
        "/app/app.zip"
    ])

    print("Searching for nRF device...")
    port = find_nrf_device()
    if not port:
        print("Could not find nRF device. Please connect it and try again.")
        return

    print(f"Device found on port {port}. Flashing firmware...")

    run_in_docker([
        "nrfutil", "dfu", "usb-serial",
        "-pkg", "/app/app.zip",
        "-p", port,
        "-b", "115200"
    ], usb=True)

if __name__ == "__main__":
    flash_with_docker()

