import json
import os

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
json_file = os.path.join(root_dir, "compile_commands.json")

with open(json_file, "r", encoding="utf-8") as f:
    data = json.load(f)

flags_to_remove = {"-fno-printf-return-value", "-mfp16-format=ieee", "-fno-reorder-functions"} # Some more could be added if necessary.

for entry in data:
    command_parts = entry["command"].split() # Remove only from "command"
    filtered_command = [part for part in command_parts if part not in flags_to_remove]
    entry["command"] = " ".join(filtered_command)

with open(json_file, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=4)

print(f"Removed specified flags from all entries in 'compile_commands.json'.")
