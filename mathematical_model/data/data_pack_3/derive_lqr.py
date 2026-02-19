import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import scipy.linalg as la

# --- CONFIGURATION ---
FILE_PATH = 'identification_signal_1/robot_identification_side_backward_2.csv'
T_START, T_END = 180.0, 215.0
CUTOFF_FREQ = 15  # Low-pass filter cutoff frequency in Hz

# LQR Weights (Tune these to change robot behavior)
# Q matrix: Penalizes state error [position, velocity, angle, angular_velocity]
Q = np.diag([10000, 1, 100000, 1])  
# R matrix: Penalizes control effort (Lower R = more aggressive PWM usage)
R = 1                       

def solve_lqr(A, B, Q, R):
    """
    Solves the continuous-time Algebraic Riccati Equation (ARE) 
    and returns the optimal gain matrix K.
    """
    # Solve the Riccati equation: A^T*P + P*A - P*B*R^-1*B^T*P + Q = 0
    P = la.solve_continuous_are(A, B, Q, R)
    
    # Compute K = inv(R) @ B^T @ P
    if np.isscalar(R):
        K = (1.0 / R) * B.T @ P
    else:
        K = np.linalg.inv(R) @ B.T @ P
    return K

def full_robot_control_design(file_path, t_start, t_end, cutoff):
    # --- 1. DATA PREPARATION ---
    df = pd.read_csv(file_path)
    df['time'] = df['dt'].cumsum()
    dt = df['dt'].mean()
    
    # Low-pass filter for noisy derivative signals
    fs = 1.0 / dt
    b, a = butter(4, cutoff / (0.5 * fs), btype='low')
    df['angle_dt_f'] = filtfilt(b, a, df['angle_dt'])
    df['pos_dt_f'] = filtfilt(b, a, df['pos_dt'])
    
    # Select the identification window
    mask = (df['time'] >= t_start) & (df['time'] <= t_end)
    df_cut = df.loc[mask].copy()
    
    # Center the angle based on the hanging equilibrium position
    angle_off = df_cut['angle'].mean()
    df_cut['angle_c'] = df_cut['angle'] - angle_off
    
    # --- 2. MODEL IDENTIFICATION (Hanging Position) ---
    X = df_cut[['pos', 'pos_dt_f', 'angle_c', 'angle_dt_f']].values
    U = df_cut[['pwm']].values
    X_k, X_k1, U_k = X[:-1, :], X[1:, :], U[:-1, :]
    
    # Least-squares identification of the discrete-time system
    Phi = np.column_stack((X_k, U_k))
    theta = np.linalg.lstsq(Phi, X_k1, rcond=None)[0].T
    A_d, B_d = theta[:, :4], theta[:, 4:].reshape(-1, 1)
    
    # Convert discrete-time model to continuous-time
    A_c = la.logm(A_d) / dt
    B_c = la.inv(A_d - np.eye(4)) @ A_c @ B_d
    
    # --- 3. MODEL INVERSION (From Hanging to Upright) ---
    # In the upright position, gravity (A[3,2]) becomes a destabilizing force (positive)
    A_upright = A_c.copy()
    A_upright[3, 2] = abs(A_c[3, 2]) # Gravity now pushes the robot down
    # The coupling between angle and linear acceleration also flips sign
    A_upright[1, 2] = -A_c[1, 2] 
    
    # --- 4. LQR CONTROL DESIGN ---
    K = solve_lqr(A_upright, B_c, Q, R)
    
    print("\n" + "="*50)
    print("IDENTIFIED LQR GAIN MATRIX (K)")
    print("="*50)
    print(f"K_pos (Position):   {K[0,0]:.4f}")
    print(f"K_vel (Velocity):   {K[0,1]:.4f}")
    print(f"K_ang (Angle):      {K[0,2]:.4f}")
    print(f"K_omg (Ang. Vel):   {K[0,3]:.4f}")
    print("\nImplementation for C++:")
    print(f"pwm = -( {K[0,0]:.4f}*pos + {K[0,1]:.4f}*vel + {K[0,2]:.4f}*angle + {K[0,3]:.4f}*omega )")
    print("="*50)
    
    # --- 5. CLOSED-LOOP SIMULATION ---
    t_sim = np.linspace(0, 5, 500)
    # Initial state: [pos, vel, angle, ang_vel] -> 0.1 rad (~6 degrees) tilt
    x0 = [0, 0, 0.1, 0] 
    
    def system_dynamics(x, t):
        # State feedback control law
        u = -K @ x
        # Saturate PWM to hardware limits (±100%)
        u = np.clip(u, -100, 100)
        return (A_upright @ x + B_c.flatten() * u)

    from scipy.integrate import odeint
    sol = odeint(system_dynamics, x0, t_sim)
    
    # --- 6. PLOTTING SIMULATION RESULTS ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Angle plot
    ax1.plot(t_sim, sol[:, 2], label='Angle [rad]', color='blue', lw=2)
    ax1.axhline(0, color='black', lw=1, ls='--')
    ax1.set_title("LQR Simulation: Angle Recovery to Upright")
    ax1.set_ylabel("Angle [rad]")
    ax1.legend(); ax1.grid(True, ls=':')
    
    # Position plot
    ax2.plot(t_sim, sol[:, 0], label='Position [m]', color='green', lw=2)
    ax2.axhline(0, color='black', lw=1, ls='--')
    ax2.set_title("LQR Simulation: Position Stabilization")
    ax2.set_ylabel("Position [m]")
    ax2.set_xlabel("Time [s]")
    ax2.legend(); ax2.grid(True, ls=':')
    
    plt.tight_layout()
    plt.show()

# EXECUTE
full_robot_control_design(FILE_PATH, T_START, T_END, CUTOFF_FREQ)
