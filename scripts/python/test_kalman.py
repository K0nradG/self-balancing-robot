import numpy as np
import matplotlib.pyplot as plt

# Set the random seed for reproducibility
np.random.seed(42)

# Time step:
dt = 0.001
time = np.arange(0, 1000, dt)

# Simulated true angle (sine wave, just for demonstration)
true_angle = np.sin(time)

# Simulated accelerometer data (just noisy version of the true angle)
accel_angle = true_angle + np.random.normal(0, 0.05, size=len(time))  # Add noise to accelerometer

# Simulated gyroscope data (angular velocity with drift)
gyro_drift = 0.01  # Initial gyroscope drift (bias)
gyro_drift_walk = 0.0001 # Random walk factor for drift (small change per step)
gyro_rate = np.cos(time) + gyro_drift  # Angular velocity with added drift

# Initialize the gyroscope angle (integrated from the angular velocity)
gyro_angle = np.zeros(len(time))

# Kalman Filter Variables
x = 0.0      # Initial estimate of the angle
P = 1.0      # Initial covariance (uncertainty)
Q = 1.0      # Process noise covariance (uncertainty in gyro prediction)
R = 0.1     # Measurement noise covariance (uncertainty in accelerometer measurement)

# Store results
estimates = []

# Kalman Filter implementation
for i in range(1, len(time)):
    # Simulate the drift walking over time
    gyro_drift += np.random.normal(0, gyro_drift_walk)  # Apply random walk to drift

    # Update gyroscope angle with drift and angular velocity (integrating angular velocity)
    gyro_angle[i] = gyro_angle[i-1] + (gyro_rate[i] + gyro_drift) * dt  # Integrate to get angle

    # Prediction step: Use the current gyroscope angle directly for prediction
    x = gyro_angle[i]  # Use the gyroscope angle as the prediction

    # Prediction covariance
    P += Q  # Increase uncertainty based on process noise

    # Kalman Gain Calculation
    K = P / (P + R)  # Calculate Kalman gain

    # Update step: Correct the prediction using the accelerometer
    x += K * (accel_angle[i] - x)  # Correct predicted angle using measurement

    # Update covariance
    P = (1 - K) * P  # Update uncertainty

    # Store the estimate
    estimates.append(x)

# Convert estimates to a numpy array for easier plotting
estimates = np.array(estimates)

# Plotting the results
plt.figure(figsize=(10, 6))
plt.plot(time, true_angle, label='True Angle (Accelerometer)', color='g', linestyle='--')
plt.plot(time, accel_angle, label='Accelerometer Angle (Noisy)', color='r', alpha=0.6)
plt.plot(time, gyro_angle, label='Gyroscope Angle (With Drift)', color='orange', linestyle='-.')
plt.plot(time[1:], estimates, label='Kalman Filter Estimate', color='b')  # Skip first estimate (it was uninitialized)
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Angle Estimation with Gyroscope Drift Using Kalman Filter')
plt.legend()
plt.grid(True)
plt.show()
