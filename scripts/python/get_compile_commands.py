import json
import os
import shutil

def update_compile_commands():
    root_dir: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    root_json: str = os.path.join(root_dir, "compile_commands.json")
    app_json: str = os.path.join(root_dir, "app", "build", "app", "compile_commands.json")
    
    if not os.path.exists(app_json):
        print(f"Error: {app_json} does not exist.")
        return
    
    if not os.path.exists(root_json):
        print(f"Warning: {root_json} does not exist. Creating an empty file.")
        with open(root_json, "w", encoding="utf-8") as f:
            f.write("[]")
    
    shutil.copy(app_json, root_json)
    print("Updated compile_commands.json successfully.")

def format_compile_commands():
    root_dir: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    json_file: str = os.path.join(root_dir, "compile_commands.json")
    
    try:
        with open(json_file, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        print("Error: compile_commands.json not found or invalid JSON.")
        return
    
    flags_to_remove = {"-fno-printf-return-value", "-mfp16-format=ieee", "-fno-reorder-functions"}
    
    for entry in data:
        command_parts = entry["command"].split()
        filtered_command = [part for part in command_parts if part not in flags_to_remove]
        entry["command"] = " ".join(filtered_command)
    
    with open(json_file, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
    
    print("Formatted compile_commands.json successfully.")

def main():
    print("Running update function...")
    update_compile_commands()
    
    print("Running format function...")
    format_compile_commands()
    
    print("All tasks completed successfully!")

if __name__ == "__main__":
    main()