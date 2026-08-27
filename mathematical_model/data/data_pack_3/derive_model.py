import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import scipy.linalg as la

# --- CONFIGURATION ---
FILE_PATH = "identification_signal_1/robot_identification_side_backward_2.csv"
T_START = 180.0
T_END = 215.0
CUTOFF_FREQ = 15  # Hz


def plot_full_state_comparison(file_path, t_start, t_end, cutoff):
    # 1. Read and Filter Data
    df = pd.read_csv(file_path)
    df["time"] = df["dt"].cumsum()
    dt = df["dt"].mean()

    fs = 1.0 / dt
    b, a = butter(4, cutoff / (0.5 * fs), btype="low")

    # Prepare real data (with filtering for velocity/derivatives)
    df["angle_dt_f"] = filtfilt(b, a, df["angle_dt"])
    df["pos_dt_f"] = filtfilt(b, a, df["pos_dt"])

    # Slice the time window
    mask = (df["time"] >= t_start) & (df["time"] <= t_end)
    df_cut = df.loc[mask].copy()

    # Angle Offset (hanging pendulum equilibrium point)
    angle_off = df_cut["angle"].mean()
    df_cut["angle_c"] = df_cut["angle"] - angle_off

    # 2. Model Identification (Discrete Time)
    X = df_cut[["pos", "pos_dt_f", "angle_c", "angle_dt_f"]].values
    U = df_cut[["pwm"]].values

    X_k = X[:-1, :]
    X_k1 = X[1:, :]
    U_k = U[:-1, :]

    Phi = np.column_stack((X_k, U_k))
    # Least-Squares solution
    theta_t, _, _, _ = np.linalg.lstsq(Phi, X_k1, rcond=None)
    theta = theta_t.T
    A_d = theta[:, :4]
    B_d = theta[:, 4:].reshape(-1, 1)

    # 3. Convert to Continuous Time (A_c, B_c)
    # This represents the actual differential equations: dx/dt = Ax + Bu
    A_c = la.logm(A_d) / dt
    B_c = la.inv(A_d - np.eye(4)) @ A_c @ B_d

    # --- PRINTING MATRICES ---
    np.set_printoptions(suppress=True, precision=4)
    print("\n" + "=" * 50)
    print("IDENTIFIED MODEL MATRICES")
    print("=" * 50)
    print("\nDiscrete-Time Matrix A_d (System Dynamics):")
    print(A_d)
    print("\nDiscrete-Time Matrix B_d (Input Influence):")
    print(B_d)
    print("-" * 50)
    print("\nContinuous-Time Matrix A_c (Physical Dynamics):")
    print(A_c.real)  # Using .real to discard negligible imaginary parts from logm
    print("\nContinuous-Time Matrix B_c (Physical Input):")
    print(B_c.real)
    print("=" * 50 + "\n")

    # 4. Model Simulation
    x_sim = np.zeros_like(X)
    x_sim[0] = X[0]
    for k in range(len(df_cut) - 1):
        x_sim[k + 1] = A_d @ x_sim[k] + B_d.flatten() * U[k]

    # 5. Full State Comparison Plot
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    time = df_cut["time"]

    labels = [
        ("Position [m]", X[:, 0], x_sim[:, 0], "forestgreen"),
        ("Linear Velocity [m/s]", X[:, 1], x_sim[:, 1], "darkorange"),
        ("Angle [rad]", X[:, 2], x_sim[:, 2], "royalblue"),
        ("Angular Velocity [rad/s]", X[:, 3], x_sim[:, 3], "firebrick"),
    ]

    for i, (title, real, sim, color) in enumerate(labels):
        axes[i].plot(time, real, label="Real Data", color=color, alpha=0.4, lw=2)
        axes[i].plot(time, sim, "--", label="Model Prediction", color="black", lw=1.5)
        axes[i].set_ylabel(title)
        axes[i].legend(loc="upper right")
        axes[i].grid(True, ls=":")

    # PWM Control Plot
    axes[4].step(time, U, color="purple", label="PWM [%]")
    axes[4].set_ylabel("Control [PWM %]")
    axes[4].set_xlabel("Time [s]")
    axes[4].grid(True, ls=":")
    axes[4].legend(loc="upper right")

    plt.suptitle(
        f"Full Model State Validation\n(Window: {t_start}s - {t_end}s)", fontsize=16
    )
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

    return A_d, B_d


# Execution:
A, B = plot_full_state_comparison(FILE_PATH, T_START, T_END, CUTOFF_FREQ)
