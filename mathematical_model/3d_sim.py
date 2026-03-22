import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.transforms import Affine2D
import matplotlib.patches as patches
import control
from collections import deque
import os

from model_identification import run_full_process

# ==========================================
# CONFIGURATION
# ==========================================
CSV_FILE = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
TS = 0.005520796680457168 
X0 = [0.0, 0.0, -0.15, 0.0]

# LQR Tuning
Q = np.diag([2000, 100, 50000, 500])
R = np.array([[100]])

# Disturbances configuration
DISTURBANCE_PUSH = 0.15 # Push magnitude (affects angular velocity)

def export_k_matrix(K, filename="lqr_config"):
    """
    Exports the K matrix to .py and .h files for real-world use.
    Now defined OUTSIDE the class to fix NameError.
    """
    with open(f"{filename}.py", "w") as f:
        f.write("# LQR Gain Matrix generated from simulation\n")
        f.write("import numpy as np\n\n")
        f.write(f"K = np.array({K.tolist()})\n")
    
    with open(f"{filename}.h", "w") as f:
        f.write("// LQR Gain Matrix for C++/Arduino\n")
        f.write("#ifndef LQR_CONFIG_H\n#define LQR_CONFIG_H\n\n")
        k_str = ", ".join([f"{val:.6f}f" for val in K])
        f.write(f"const float K_LQR[4] = {{{k_str}}};\n\n")
        f.write("#endif\n")
    
    print(f"\n[SUCCESS] Matrix K exported to {filename}.py and {filename}.h")

class LiveRobotSim:
    def __init__(self, A_d, B_d, K):
        self.A_d = A_d
        self.B_d = B_d
        self.K = K
        
        # State & History
        self.x = np.array(X0)
        self.time = 0.0
        self.push_dir = 1.0 # 1.0 or -1.0
        
        # Fixed-length history for plotting (keeps simulation fast)
        max_len = 1000
        self.time_hist = deque([0.0], maxlen=max_len)
        self.tilt_hist = deque([self.x[2]], maxlen=max_len)
        self.u_hist = deque([0.0], maxlen=max_len)

        # UI Setup
        self.fig = plt.figure(figsize=(12, 7))
        self.fig.canvas.manager.set_window_title('Interactive LQR Self-Balancing Robot')
        
        # Connect keyboard listener
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        # Subplot 1: Live Animation
        self.ax_sim = plt.subplot2grid((2, 2), (0, 0), rowspan=2, aspect='equal')
        self.ax_sim.set_xlim(-0.5, 0.5)
        self.ax_sim.set_ylim(-0.1, 0.4)
        self.ax_sim.set_xlabel('Position [m]')
        self.ax_sim.grid(True, ls=':')
        
        # Drawing elements
        self.wheel_r = 0.05
        self.wheel = plt.Circle((0, self.wheel_r), self.wheel_r, fc='orange', ec='k', zorder=10)
        self.body = patches.Rectangle((0, 0), 0.06, 0.2, fc='blue', ec='k', zorder=9)
        self.ax_sim.add_patch(self.wheel)
        self.ax_sim.add_patch(self.body)
        self.ax_sim.axhline(0, color='k', lw=2)

        # On-screen HUD
        self.txt_time = self.ax_sim.text(-0.45, 0.36, '', family='monospace')
        self.txt_tilt = self.ax_sim.text(-0.45, 0.32, '', color='blue', family='monospace')
        self.txt_pos = self.ax_sim.text(-0.45, 0.28, '', color='red', family='monospace')
        self.txt_info = self.ax_sim.text(-0.45, -0.05, 'Press "z" to PUSH the robot', color='purple', fontweight='bold')

        # Subplot 2: Tilt graph
        self.ax_angle = plt.subplot2grid((2, 2), (0, 1))
        self.ax_angle.set_title('Tilt Angle')
        self.ax_angle.grid(True, ls=':')
        self.line_angle, = self.ax_angle.plot([], [], 'g-')
        self.ax_angle.axhline(0, color='k', ls='--', lw=0.5)

        # Subplot 3: PWM graph
        self.ax_pwm = plt.subplot2grid((2, 2), (1, 1))
        self.ax_pwm.set_title('Control PWM')
        self.ax_pwm.set_xlabel('Time [s]')
        self.ax_pwm.grid(True, ls=':')
        self.line_pwm, = self.ax_pwm.plot([], [], 'r-')
        self.ax_pwm.set_ylim(-110, 110)

    def on_key(self, event):
        """Keyboard disturbance handler."""
        if event.key == 'z':
            # Adds angular velocity impulse and flips direction
            self.x[3] += self.push_dir * DISTURBANCE_PUSH
            self.push_dir *= -1.0
            print(f"Robot Pushed! Impulse applied: {self.push_dir * -DISTURBANCE_PUSH:+.2f}")

    def update(self, frame):
        # 1. Physics Step (LQR control & state space)
        u = -self.K @ self.x
        u = np.clip(u, -100, 100) # PWM Saturation
        
        self.x = self.A_d @ self.x + self.B_d.flatten() * u
        self.time += TS

        # 2. Save history for graphs
        self.time_hist.append(self.time)
        self.tilt_hist.append(self.x[2])
        self.u_hist.append(u)

        # 3. Update Visuals (Animation pane)
        pos, tilt = self.x[0], self.x[2]
        self.wheel.set_center((pos, self.wheel_r))
        
        # Center body on top of wheel and rotate
        tr = Affine2D().rotate_around(pos, self.wheel_r, -tilt)
        self.body.set_transform(tr + self.ax_sim.transData)
        self.body.set_xy((pos - 0.03, self.wheel_r))
        
        # Follow camera
        self.ax_sim.set_xlim(pos - 0.5, pos + 0.5)

        # 4. Update HUD
        self.txt_time.set_text(f'Time: {self.time:.2f}s')
        self.txt_tilt.set_text(f'Tilt: {tilt:+.4f} rad')
        self.txt_pos.set_text(f'Pos: {pos:+.3f} m')

        # 5. Update right-side plots
        t_arr, tilt_arr, u_arr = list(self.time_hist), list(self.tilt_hist), list(self.u_hist)
        self.line_angle.set_data(t_arr, tilt_arr)
        self.line_pwm.set_data(t_arr, u_arr)
        
        self.ax_angle.set_xlim(t_arr[0], t_arr[-1])
        self.ax_angle.set_ylim(min(tilt_arr) - 0.1, max(tilt_arr) + 0.1)
        self.ax_pwm.set_xlim(t_arr[0], t_arr[-1])

        return self.wheel, self.body, self.txt_time, self.txt_tilt, self.txt_pos, self.line_angle, self.line_pwm

    def run(self):
        # High FPS simulation run
        self.ani = FuncAnimation(self.fig, self.update, interval=1, blit=False)
        plt.show()


if __name__ == "__main__":
    # 1. Load identified model
    model = run_full_process(CSV_FILE, 150.0, 168.0, db=3.0)

    # 2. Design Discrete LQR
    K, _, _ = control.dlqr(model.A_d, model.B_d, Q, R)
    K = K.flatten()

    # 3. Run interactive live loop
    sim = LiveRobotSim(model.A_d, model.B_d, K)
    sim.run()

    # Export after closing the window
    ans = input("Do you want to export the K matrix to files? (y/n): ")
    if ans.lower() == 'y':
        export_k_matrix(K)

