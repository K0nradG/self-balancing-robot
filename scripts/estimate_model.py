import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import minimize, curve_fit
from typing import Tuple, Dict
import os
import re

# Function to parse serial data
def parse_serial_data(file_path: str) -> Dict[int, np.ndarray]:
    buffer_dict = {}
    current_buffer = None
    
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            
            if not line:
                continue
            
            buffer_match = re.match(r'\[INF\] MODEL: buffor data: (\d+)', line)
            data_match = re.match(r'\[INF\] MODEL: (-?\d+\.\d+)', line)
            
            if buffer_match:
                current_buffer = int(buffer_match.group(1))
                if current_buffer not in buffer_dict:
                    buffer_dict[current_buffer] = []
            elif data_match and current_buffer is not None:
                buffer_dict[current_buffer].append(float(data_match.group(1)))
    
    return {key: np.array(value) for key, value in buffer_dict.items()}

# Loading data
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, "data", "minicom.txt")
data = parse_serial_data(file_path)

tab1 = data[0]
tab1 = [x + 90 for x in tab1]  # Shift by 90 degrees
tab2 = data[1]
tab3 = data[2]
tab4 = np.linspace(0.0, 10.0,10000)

angle = np.radians(tab1) # Convert to radians
angle_dot = tab2
M_max = 0.0784
u = (tab3/100) * M_max # Convert to [kg * m^2]
time = tab4  # time in s

def model_identification(t, theta, theta_dot, u):
    # Calculate the angular acceleration (numerical differentiation)
    dt = np.diff(t)
    theta_ddot = np.diff(theta_dot) / dt

    #Remove the last element to match the lengths of the vectors
    theta = theta[:-1]
    theta_dot = theta_dot[:-1]
    u = u[:-1]

    # Create the regression matrix Y = [theta_dot, sin(theta), u]
    Y = np.vstack([theta_dot, np.sin(theta), u]).T
    theta_ddot = theta_ddot.reshape(-1, 1)

    # Parameter identification using the least squares method
    params, _, _, _ = np.linalg.lstsq(Y, theta_ddot, rcond=None)
    alpha_1, alpha_2, alpha_3 = params.flatten()

    print(f"Identified parameters:")
    print(f"α1 (damping): {alpha_1:.4f}")
    print(f"α2 (gravity): {alpha_2:.4f}")
    print(f"α3 (control): {alpha_3:.4f}")

    t_values = np.linspace(t[0], t[-1], len(u))

    def model(t, x, t_values, u):
        theta, theta_dot = x
        # Interpolate control input u at the current time t
        theta_ddot = alpha_1 * theta_dot + alpha_2 * np.sin(theta) + alpha_3 * np.interp(t, t_values, u)
        return [theta_dot, theta_ddot]

    # Solve the differential equation
    x0 = [theta[0], theta_dot[0]]

    # Solve the differential equation using solve_ivp
    sol = solve_ivp(model, [t[0], t[-1]], x0, t_eval=t, args=(t_values, u))

    # Plot comparing real and modeled angle
    plt.figure(figsize=(10, 5))
    plt.plot(time, angle, label="Real angle (measurement)", linestyle='dashed')
    plt.plot(sol.t, sol.y[0], label="Modeled angle", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")
    plt.title("Comparison of real and modeled angles")
    plt.legend()
    plt.grid()
    plt.show()

    return alpha_1, alpha_2, alpha_3

model_identification(time, angle, angle_dot, u)

