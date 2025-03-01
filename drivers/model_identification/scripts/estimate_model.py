import re
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

def parse_serial_data(file_path):
    buffer_dict = {}
    current_buffer = None
    
    with open(file_path, 'r') as file:
        for line in file:
            buffer_match = re.search(r'\[INF\] MODEL: buffor data: (\d+)', line)
            data_match = re.search(r'\[INF\] MODEL: (\d+\.\d+)', line)
            
            if buffer_match:
                current_buffer = int(buffer_match.group(1))
                buffer_dict[current_buffer] = []
            elif data_match and current_buffer is not None:
                buffer_dict[current_buffer].append(float(data_match.group(1)))
    
    return buffer_dict

def estimate_parameters(theta, theta_dot, u, dt):
    """
    Estimates the parameters of an inverted pendulum model:
    θ'' = -α * θ + β * u

    Parameters:
    theta (array): Angle displacement in radians
    theta_dot (array): Angular velocity in rad/s
    u (array): Control torque
    dt (array): Sampling time

    Returns:
    alpha (float): Parameter related to gravity and moment of inertia
    beta (float): Parameter related to control influence
    """
    # calculate angular acceleration
    theta_ddot = np.diff(theta_dot) / dt
    
    # Fitting to the model ddot_theta = -alpha * theta + beta * u
    X = np.column_stack((-theta[:-1], u[:-1]))  # Ensure that the lengths of X and y match
    y = theta_ddot  # Target values
    
    if X.shape[0] != y.shape[0]:
        min_len = min(X.shape[0], y.shape[0])
        X = X[:min_len]
        y = y[:min_len]

    #Linear regression (least squares method)
    model = LinearRegression(fit_intercept=False)
    model.fit(X, y)
    

    alpha, beta = model.coef_
    return alpha, beta



file_path = "data/minicom.txt"

data = parse_serial_data(file_path)

required_buffers = [0, 1, 2, 3]
if all(buf in data for buf in required_buffers):
    theta = np.array(data[0])
    theta_dot = np.array(data[1])
    u = np.array(data[2])
    dt = np.mean(np.diff(np.array(data[3])))

    alpha_est, beta_est = estimate_parameters(theta, theta_dot, u, dt)
    print(f"Estimated parameters: alpha = {alpha_est:.4f}, beta = {beta_est:.4f}")
else:  
    print("Missing required data in the file.")
