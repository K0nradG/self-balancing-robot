import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import scipy.linalg as la

# --- CONFIGURATION ---
FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
T_START, T_END = 180.0, 215.0
CUTOFF_FREQ = 15  # Low-pass filter cutoff frequency in Hz

Q = np.diag([10000, 1, 100000, 1])  
R = 1                       

def solve_lqr(A, B, Q, R):
    P = la.solve_continuous_are(A, B, Q, R)
    
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
    
    fs = 1.0 / dt
    b, a = butter(4, cutoff / (0.5 * fs), btype='low')
    df['angle_dt_f'] = filtfilt(b, a, df['angle_dt'])
    df['pos_dt_f'] = filtfilt(b, a, df['pos_dt'])
    
    mask = (df['time'] >= t_start) & (df['time'] <= t_end)
    df_cut = df.loc[mask].copy()
    
    angle_off = df_cut['angle'].mean()
    df_cut['angle_c'] = df_cut['angle'] - angle_off
    
    # --- 2. MODEL IDENTIFICATION ---
    X = df_cut[['pos', 'pos_dt_f', 'angle_c', 'angle_dt_f']].values
    U = df_cut[['pwm']].values
    
    X_k, X_k1, U_k = X[:-1, :], X[1:, :], U[:-1, :]
    
    Phi = np.column_stack((X_k, U_k))
    theta = np.linalg.lstsq(Phi, X_k1, rcond=None)[0].T
    A_d = theta[:, :4]
    B_d = theta[:, 4:].reshape(-1, 1)
    
    A_c = la.logm(A_d) / dt
    B_c = la.inv(A_d - np.eye(4)) @ A_c @ B_d
    
    # --- 3. Upright model ---
    A_upright = A_c.copy()
    A_upright[3, 2] = abs(A_c[3, 2])
    A_upright[1, 2] = -A_c[1, 2]
    
    # --- 4. LQR ---
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
    
    # --- 5. CLOSED-LOOP SIMULATION (Euler) ---
    T_sim = 5.0
    dt_sim = 0.001
    steps = int(T_sim / dt_sim)

    x = np.array([0, 0, 0.4, 0])

    X_history = np.zeros((steps, 4))
    U_history = np.zeros(steps)
    t_sim = np.linspace(0, T_sim, steps)

    for i in range(steps):

        u = -K @ x
        u = np.clip(u, -100, 100)

        X_history[i, :] = x
        U_history[i] = u

        x_dot = A_upright @ x + B_c.flatten() * u
        x = x + x_dot * dt_sim

    # --- 6. PLOTS ---
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
    
    # Angle plot (blue jak było)
    ax1.plot(t_sim, X_history[:, 2], label='Angle [rad]', color='blue', lw=2)
    ax1.axhline(0, color='black', lw=1, ls='--')
    ax1.set_title("LQR Simulation: Angle Recovery to Upright")
    ax1.set_ylabel("Angle [rad]")
    ax1.legend(); ax1.grid(True, ls=':')
    
    # Position plot (green jak było)
    ax2.plot(t_sim, X_history[:, 0], label='Position [m]', color='green', lw=2)
    ax2.axhline(0, color='black', lw=1, ls='--')
    ax2.set_title("LQR Simulation: Position Stabilization")
    ax2.set_ylabel("Position [m]")
    ax2.legend(); ax2.grid(True, ls=':')

    # PWM plot (czarny – neutralny)
    ax3.plot(t_sim, U_history, label='PWM [%]', color='black', lw=2)
    ax3.axhline(100, color='black', lw=1, ls='--')
    ax3.axhline(-100, color='black', lw=1, ls='--')
    ax3.set_title("Control Signal (PWM)")
    ax3.set_ylabel("PWM [%]")
    ax3.set_xlabel("Time [s]")
    ax3.legend(); ax3.grid(True, ls=':')

    plt.tight_layout()
    plt.show()


# EXECUTE
full_robot_control_design(FILE_PATH, T_START, T_END, CUTOFF_FREQ)
