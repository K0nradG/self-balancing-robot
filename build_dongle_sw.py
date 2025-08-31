import os
import subprocess
import sys

def run_nrfdongle_build_command():
    target_dir = os.path.join("Sources", "applications", "app")
    os.chdir(target_dir)

    west_cmd = ["west", "build", "-b", "nrf52840dongle_nrf52840", "-p"]
    try:
        subprocess.run(west_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"West command failed with exit code {e.returncode}")
        sys.exit(e.returncode)

if __name__ == "__main__":
    run_nrfdongle_build_command()