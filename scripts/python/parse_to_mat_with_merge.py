import scipy.io
import numpy as np
import re
import os
from collections import defaultdict

def merge_serial_data_to_mat() -> None:
    script_dir: str = os.path.dirname(os.path.abspath(__file__))
    parent_dir: str = os.path.dirname(script_dir)
    log_dir: str = os.path.join(script_dir, "data")
    matlab_dir: str = os.path.join(parent_dir, "matlab", "matlab_data")

    if not os.path.exists(matlab_dir):
        os.makedirs(matlab_dir)

    output_mat_file: str = os.path.join(matlab_dir, "merged_robot_data_k650.3_s-95.mat")
    files_to_merge = [
        "k650.3s-95.txt",
        "k650.3s-95-v2.txt",
        "k650.3s-95-v3.txt",
        "k650.3s-95-v4.txt"
    ]

    buffer_pattern: str = re.compile(r"MODEL:\s*buffor data:\s*(\d+)")
    value_pattern: str = re.compile(r"MODEL:\s*(-?\d+\.?\d*)")

    buffer_data = defaultdict(list)

    for filename in files_to_merge:
        log_file = os.path.join(log_dir, filename)

        if not os.path.exists(log_file):
            print(f"⚠️ Warning: {filename} not found, skipping.")
            continue

        with open(log_file, "r") as file:
            current_buffer = None
            for line in file:
                line = line.strip()

                buffer_match = buffer_pattern.search(line)
                if buffer_match:
                    current_buffer = int(buffer_match.group(1))
                    continue

                value_match = value_pattern.search(line)
                if value_match and current_buffer is not None:
                    buffer_data[current_buffer].append(float(value_match.group(1)))

    for key in buffer_data:
        buffer_data[key] = np.array(buffer_data[key])

    min_length = min(len(data) for data in buffer_data.values())

    for key in buffer_data:
        buffer_data[key] = buffer_data[key][:min_length]

    merged_data = {
        "theta": buffer_data[0] if 0 in buffer_data else np.array([]),
        "theta_dot": buffer_data[1] if 1 in buffer_data else np.array([]),
        "pwm": buffer_data[2] if 2 in buffer_data else np.array([]),
    }

    scipy.io.savemat(output_mat_file, merged_data)

    print(f"✅ Merged data successfully saved to {output_mat_file}")


if __name__ == "__main__":
    merge_serial_data_to_mat()