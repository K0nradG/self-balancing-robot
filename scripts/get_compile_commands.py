import subprocess
import os

root_dir = os.path.dirname(os.path.abspath(__file__))
update_script = os.path.join(root_dir, "update_compile_commands.py")
format_script = os.path.join(root_dir, "format_compile_commands.py")

try:
    print(f"Running update script...")
    subprocess.run(["python", update_script], check=True)
    print(f"Successfully updated compile_commands.json!")
except subprocess.CalledProcessError:
    print(f"Error running update!")

try:
    print(f"Running format script...")
    subprocess.run(["python", format_script], check=True)
    print(f"Successfully formatted compile_commands.json!")
except subprocess.CalledProcessError:
    print(f"Error running format!")
