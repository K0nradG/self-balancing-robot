# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Script allowing to flash the nrf52840 dongle device with the use of external debugger (nrf7002dk in this case) with pre-built firmware.

import os
import subprocess
import sys

def run_dongle_flash():
    target_dir = os.path.join("Sources", "applications", "app")
    original_dir = os.getcwd()

    try:
        os.chdir(target_dir)
        flash_cmd = ["west", "flash", "--erase", "--skip-rebuild"]
        subprocess.run(flash_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"West flash command failed with exit code {e.returncode}")
        sys.exit(e.returncode)
    finally:
        # Always return to the original directory
        os.chdir(original_dir)

if __name__ == "__main__":
    run_dongle_flash()