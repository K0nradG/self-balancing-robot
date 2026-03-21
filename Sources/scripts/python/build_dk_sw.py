# Copyright 2026 Filip Dymczyk and Konrad Grucel

import os
import subprocess
import sys
from Sources.scripts.python.format_changed_files import format_changed_files

def run_dk_build_command():
    target_dir = os.path.join("Sources", "applications", "controller")
    os.chdir(target_dir)

    west_cmd = ["west", "build", "-b", "nrf7002dk/nrf5340/cpuapp", "-p"]
    try:
        subprocess.run(west_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"West command failed with exit code {e.returncode}")
        sys.exit(e.returncode)

if __name__ == "__main__":
    format_changed_files
    run_dk_build_command()