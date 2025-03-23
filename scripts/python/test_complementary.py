import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# Time step
dt = 0.001  # 1ms time step
time = np.arange(0, 1000, dt)  # Simulate for 100 seconds

# Simulated true angle (sine wave, just for demonstration)
true_angle = np.sin(time)

# Simulated accelerometer data (just noisy version of the true angle)
accel_angle = true_angle + np.random.normal(0, 0.05, size=len(time))  # Add noise to accelerometer

# Simulated gyroscope data (angular velocity with drift)
gyro_drift = 0.01  # Initial gyroscope drift (bias)
gyro_drift_walk = 0.0001  # Random walk factor for drift (small change per step)
gyro_rate = np.cos(time) + gyro_drift  # Angular velocity with added drift

# Complementary Filter Variables
alpha = 0.9  # Complementary filter coefficient (you can adjust this value)
comp_angle = np.zeros(len(time))  # To store the filtered angle estimate
comp_angle[0] = accel_angle[0]  # Initialize with accelerometer angle at the start
gyro_angle = np.zeros(len(time))

# Apply the complementary filter
for i in range(1, len(time)):
    # Update gyroscope angle with drift and angular velocity (integrating angular velocity)
    gyro_angle[i] = gyro_angle[i-1] + (gyro_rate[i] + gyro_drift) * dt  # Integrate to get angle

    # Complementary filter: blending accelerometer and gyroscope angles
    comp_angle[i] = alpha * gyro_angle[i] + (1 - alpha) * accel_angle[i]

    # Simulate the drift walking over time
    gyro_drift += np.random.normal(0, gyro_drift_walk)  # Apply random walk to drift

# Plotting the results
plt.figure(figsize=(10, 6))
plt.plot(time, true_angle, label='True Angle (Accelerometer)', color='g', linestyle='--')
plt.plot(time, accel_angle, label='Accelerometer Angle (Noisy)', color='r', alpha=0.6)
plt.plot(time, gyro_angle, label='Gyroscope Angle', color='orange', linestyle='-.')
plt.plot(time, comp_angle, label='Complementary Filter Estimate', color='b')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title(f'Angle Estimation Using Complementary Filter (Alpha={alpha})')
plt.legend()
plt.grid(True)
plt.show()
