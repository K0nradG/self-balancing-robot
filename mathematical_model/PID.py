import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# --- CONFIGURATION ---
FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
T_START, T_END = 180.0, 215.0
CUTOFF_FREQ = 15

KP = 40
KI = 40
KD = 3.0

HYSTERESIS = 0.0
DT_MIN = 0.001
DT_MAX = 0.05
ABS_DIFF = 1e-3


# ---------------- FILTER ----------------
class LowPassFilter:
    def __init__(self, alpha=1.0):
        self.alpha = alpha
        self.state = 0.0

    def filter(self, value):
        self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state


# ---------------- PID 1:1 ----------------
class PID:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.integral = 0.0
        self.prev_error = 0.0
        self.filter = LowPassFilter(alpha=1.0)
        self.output_limit = 100.0

    def calculate_output(self, setpoint, feedback, dt):

        dt = max(min(dt, DT_MAX), DT_MIN)

        error = setpoint - self.filter.filter(feedback)

        if abs(error) < HYSTERESIS:
            self.integral = 0.0
            self.prev_error = error
            return 0.0

        if abs(self.Ki) > ABS_DIFF:
            self.integral += self.Ki * error * dt
        else:
            self.integral = 0.0

        derivative = self.Kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = self.Kp * error + self.integral + derivative
        saturated_output = np.clip(output, -self.output_limit, self.output_limit)

        # Anti-windup
        if (abs(output - saturated_output) > ABS_DIFF) and ((output * error) > 0):
            self.integral -= self.Ki * error * dt

        return saturated_output


# ---------------- MAIN ----------------
def full_robot_pid_simulation(file_path, t_start, t_end, cutoff):

    # --- IDENTIFICATION (DISCRETE MODEL) ---
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

    X = df_cut[['pos', 'pos_dt_f', 'angle_c', 'angle_dt_f']].values
    U = df_cut[['pwm']].values

    X_k = X[:-1, :]
    X_k1 = X[1:, :]
    U_k = U[:-1, :]

    Phi = np.column_stack((X_k, U_k))
    theta = np.linalg.lstsq(Phi, X_k1, rcond=None)[0].T

    A_d = theta[:, :4]
    B_d = theta[:, 4:].reshape(-1, 1)

    print("Eigenvalues A_d:")
    print(np.linalg.eigvals(A_d))

    # --- SYMULACJA (DYSKRETNA) ---
    T_sim = 10
    dt_sim = 0.002  # używamy dokładnie tego samego kroku co identyfikacja
    steps = int(T_sim / dt_sim)

    x = np.array([0, 0, 0.3, 0])

    X_history = np.zeros((steps, 4))
    U_history = np.zeros(steps)
    t_sim = np.linspace(0, T_sim, steps)

    pid = PID(KP, KI, KD)

    for i in range(steps):

        angle = x[2]

        u = pid.calculate_output(0, angle, dt_sim)
        U_history[i] = u

        # 🔥 DISCRETE UPDATE (bez Euler, bez logm)
        x = A_d @ x + B_d.flatten() * u

        X_history[i] = x

    # --- PLOTS ---
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))

    ax1.plot(t_sim, X_history[:, 2], label='Angle [rad]', color='blue', lw=2)
    ax1.axhline(0, color='black', lw=1, ls='--')
    ax1.set_title("PID 1:1 Simulation: Angle Recovery")
    ax1.set_ylabel("Angle [rad]")
    ax1.legend(); ax1.grid(True, ls=':')

    ax2.plot(t_sim, X_history[:, 0], label='Position [m]', color='green', lw=2)
    ax2.axhline(0, color='black', lw=1, ls='--')
    ax2.set_title("Position Response")
    ax2.set_ylabel("Position [m]")
    ax2.legend(); ax2.grid(True, ls=':')

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
full_robot_pid_simulation(FILE_PATH, T_START, T_END, CUTOFF_FREQ)
