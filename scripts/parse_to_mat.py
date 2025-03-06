import scipy.io
import numpy as np
import re
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
log_file = os.path.join(script_dir, "data", "minicom_v2.txt")
matlab_dir = os.path.join(script_dir, "matlab_data")

if not os.path.exists(matlab_dir):
    os.makedirs(matlab_dir)

output_mat_file = os.path.join(matlab_dir, "robot_data_v2.mat")

with open(log_file, "r") as file:
    lines = [line.strip() for line in file if "MODEL:" in line]

values = []
for line in lines:
    match = re.search(r"MODEL:\s*(-?\d+\.?\d*)", line)
    if match:
        values.append(float(match.group(1)))

if len(values) == 0:
    raise ValueError("No valid 'MODEL:' data found in the input file.")

total_entries = len(values)
print(total_entries)
buffer_size = total_entries // 4

angles = np.array(values[:buffer_size])
angle_dt = np.array(values[buffer_size:2*buffer_size])
pwm_values = np.array(values[2*buffer_size:]) / 100

min_length = min(len(angles), len(angle_dt), len(pwm_values))

angles = angles[:min_length]
angle_dt = angle_dt[:min_length]
pwm_values = pwm_values[:min_length]

scipy.io.savemat(output_mat_file, {
    "theta": angles,
    "theta_dot": angle_dt,
    "pwm": pwm_values
})

print(f"✅ Data successfully saved to {output_mat_file}")