import os
import shutil

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
root_json = os.path.join(root_dir, "compile_commands.json")
app_json = os.path.join(root_dir, "app", "build", "app", "compile_commands.json")

if not os.path.exists(app_json):
    print(f"Error: {app_json} does not exist.")
    exit(1)

if not os.path.exists(root_json):
    print(f"Warning: {root_json} does not exist. Creating an empty file.")
    with open(root_json, "w", encoding="utf-8") as f:
        f.write("[]")

shutil.copy(app_json, root_json)
