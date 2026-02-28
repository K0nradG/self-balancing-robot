import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from model_identification import run_full_process


def simulate_lqr(matrices, K, t_max=5.0):
    """Simulate continuous-time LQR closed-loop response."""
    A = matrices.A_c
    B = matrices.B_c

    # Closed-loop: dx/dt = (A - B K) x
    def closed_loop_system(x, t):
        u = -K @ x
        u = np.clip(u, -100, 100)  # PWM saturation
        return A @ x + B.flatten() * u

    # Initial condition: 0.1 rad tilt
    x0 = [0.0, 0.0, 0.1, 0.0]
    t = np.linspace(0, t_max, 500)

    x_sol = odeint(closed_loop_system, x0, t)
    u_sol = np.array([-K @ x for x in x_sol])

    return t, x_sol, u_sol


# 1. Load identified upright model
FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
matrices_upright = run_full_process(FILE_PATH, 100.0, 168.0, db=5.0)

# 2. LQR design
Q = np.diag([100, 10000, 1000, 10000])  # state weights
R = np.array([[1000]])                 # input weight

P = la.solve_continuous_are(
    matrices_upright.A_c,
    matrices_upright.B_c,
    Q,
    R
)

K = (la.inv(R) @ matrices_upright.B_c.T @ P).flatten()

# 3. Simulation
t, x, u = simulate_lqr(matrices_upright, K)

# 4. Plots
fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

axes[0].plot(t, x[:, 2], 'g', lw=2, label='Angle [rad]')
axes[0].axhline(0, color='black', lw=1, ls='--')
axes[0].set_ylabel('Angle [rad]')
axes[0].set_title('Closed-loop response to 0.1 rad initial tilt')

axes[1].plot(t, x[:, 0], 'b', lw=2, label='Position [m]')
axes[1].axhline(0, color='black', lw=1, ls='--')
axes[1].set_ylabel('Position [m]')

axes[2].plot(t, u, 'r', lw=2, label='Control input (PWM)')
axes[2].set_ylabel('PWM [%]')
axes[2].set_xlabel('Time [s]')

for ax in axes:
    ax.grid(True, ls=':')
    ax.legend()

plt.tight_layout()
plt.show()

print(f"Computed LQR gains K: {K}")
