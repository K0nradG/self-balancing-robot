# Copyright 2026 Filip Dymczyk and Konrad Grucel

# Builds the nRF52840dongle application using West build system

import os
import subprocess
import sys
from Sources.scripts.python.format_changed_files import format_changed_files
from Sources.scripts.python.get_compile_commands import get_compile_commands

def run_nrfdongle_build_command():
    target_dir = os.path.join("Sources", "applications", "app")
    original_dir = os.getcwd()

    try:
        os.chdir(target_dir)
        west_cmd = ["west", "build", "-b", "nrf52840dongle_nrf52840", "-p"]
        subprocess.run(west_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"West command failed with exit code {e.returncode}")
        sys.exit(e.returncode)
    finally:
        # Always return to the original directory
        os.chdir(original_dir)

if __name__ == "__main__":
    format_changed_files()
    run_nrfdongle_build_command()
    get_compile_commands()