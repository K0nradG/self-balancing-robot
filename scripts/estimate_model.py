import os
import re
import numpy as np
from sklearn.linear_model import LinearRegression
import string
from typing import Tuple, Dict

def parse_serial_data(file_path: string) -> Dict[int, np.ndarray[float]]:
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
    
    return buffer_dict

def estimate_parameters(theta: np.ndarray, theta_dot: np.ndarray, u: np.ndarray, dt: float) -> Tuple[float, float]:
    """
    Estimates the parameters of an inverted pendulum model:
    θ'' = -α * θ + β * u
    """
    theta_ddot = np.diff(theta_dot) / dt

    # Trim all arrays to the same length:
    min_length = min(len(theta) - 1, len(theta_dot) - 1, len(theta_ddot), len(u) - 1)
    theta = theta[:min_length]
    theta_dot = theta_dot[:min_length]
    u = u[:min_length]
    theta_ddot = theta_ddot[:min_length]

    # Prepare regression model: θ'' = -α * θ + β * u
    X = np.column_stack((-theta, u))
    y = theta_ddot

    # Fit the linear model:
    model = LinearRegression(fit_intercept=False)
    model.fit(X, y)

    alpha, beta = model.coef_
    return alpha, beta

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, "data", "minicom.txt")
    data = parse_serial_data(file_path)
    required_buffers = [0, 1, 2, 3]

    M_max = 0.0784 # [kg * m^2]
    ms_to_s = 1e-03
    if all(buf in data for buf in required_buffers):
        theta = np.array(data[0])
        theta_dot = np.array(data[1])
        u = (np.array(data[2]) / 100.0) * M_max 
        dt = np.mean(np.diff(np.array(data[3]))) * ms_to_s

        alpha_est, beta_est = estimate_parameters(theta, theta_dot, u, dt)
        print(f"Estimated parameters: alpha = {alpha_est:.4f}, beta = {beta_est:.4f}")
    else:  
        print("Missing required data in the file.")
