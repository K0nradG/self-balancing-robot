import numpy as np
import matplotlib.pyplot as plt
from model_identification import DataProcessor, ModelIdentifier, identify_model


class LQRController:
    """Linear Quadratic Regulator controller"""
    
    def __init__(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: float):
        """
        Args:
            A: continuous-time system matrix
            B: continuous-time input matrix
            Q: state cost matrix
            R: control cost
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = self._solve_lqr()
    
    def _solve_lqr(self) -> np.ndarray:
        """Solve continuous-time LQR problem"""
        from scipy.linalg import solve_continuous_are
        
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        
        if np.isscalar(self.R):
            K = (1.0 / self.R) * self.B.T @ P
        else:
            K = np.linalg.inv(self.R) @ self.B.T @ P
            
        return K
    
    def compute_control(self, state: np.ndarray) -> float:
        """Compute control input u = -K*x"""
        # Fix: properly extract scalar value from the dot product
        u = -self.K @ state
        # If u is an array with one element, extract it
        if isinstance(u, np.ndarray) and u.size == 1:
            return float(u.item())
        return float(u)
    
    def print_gains(self):
        """Print controller gains in readable format"""
        print("\n" + "="*50)
        print("LQR GAIN MATRIX (K)")
        print("="*50)
        print(f"K_pos (Position):   {self.K[0,0]:.4f}")
        print(f"K_vel (Velocity):   {self.K[0,1]:.4f}")
        print(f"K_ang (Angle):      {self.K[0,2]:.4f}")
        print(f"K_omg (Ang. Vel):   {self.K[0,3]:.4f}")
        print("\nC++ implementation:")
        print(f"pwm = -( {self.K[0,0]:.4f}*pos + {self.K[0,1]:.4f}*vel + "
              f"{self.K[0,2]:.4f}*angle + {self.K[0,3]:.4f}*omega )")
        print("="*50)


class LQRSimulator:
    """Simulates closed-loop system with LQR controller"""
    
    def __init__(self, A: np.ndarray, B: np.ndarray, controller: LQRController):
        self.A = A
        self.B = B
        self.controller = controller
    
    def simulate(self, initial_state: np.ndarray, T_sim: float = 10.0, 
                 dt: float = 0.002, u_limit: float = 100.0) -> dict:
        """
        Simulate closed-loop response
        
        Returns:
            dict with 'time', 'states', 'control' arrays
        """
        steps = int(T_sim / dt)
        t = np.linspace(0, T_sim, steps)
        
        X = np.zeros((steps, 4))
        U = np.zeros(steps)
        x = initial_state.copy()
        
        for i in range(steps):
            # Store current state
            X[i] = x
            
            # Compute and saturate control
            u = self.controller.compute_control(x)
            u = np.clip(u, -u_limit, u_limit)
            U[i] = u
            
            # Euler integration
            x_dot = self.A @ x + self.B.flatten() * u
            x = x + x_dot * dt
        
        return {
            'time': t,
            'states': X,
            'control': U
        }
    
    def plot_results(self, results: dict):
        """Plot simulation results"""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
        
        t = results['time']
        X = results['states']
        U = results['control']
        
        # Angle plot
        ax1.plot(t, X[:, 2], label='Angle [rad]', color='blue', lw=2)
        ax1.axhline(0, color='black', lw=1, ls='--')
        ax1.set_title("LQR: Angle Recovery")
        ax1.set_ylabel("Angle [rad]")
        ax1.legend()
        ax1.grid(True, ls=':')
        
        # Position plot
        ax2.plot(t, X[:, 0], label='Position [m]', color='green', lw=2)
        ax2.axhline(0, color='black', lw=1, ls='--')
        ax2.set_title("LQR: Position Stabilization")
        ax2.set_ylabel("Position [m]")
        ax2.legend()
        ax2.grid(True, ls=':')
        
        # PWM plot
        ax3.plot(t, U, label='PWM [%]', color='black', lw=2)
        ax3.axhline(100, color='black', lw=1, ls='--')
        ax3.axhline(-100, color='black', lw=1, ls='--')
        ax3.set_title("Control Signal")
        ax3.set_ylabel("PWM [%]")
        ax3.set_xlabel("Time [s]")
        ax3.legend()
        ax3.grid(True, ls=':')
        
        plt.tight_layout()
        plt.show()


# Main execution
if __name__ == "__main__":
    # Configuration
    FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
    T_START, T_END = 180.0, 215.0
    CUTOFF_FREQ = 15
    
    # LQR weights
    Q = np.diag([10000, 1, 100000, 1])
    R = 1.0
    
    # Identify model
    print("Identifying model...")
    matrices = identify_model(FILE_PATH, T_START, T_END, CUTOFF_FREQ, plot=False)
    
    # Create upright-stable model
    identifier = ModelIdentifier()
    upright_matrices = identifier.create_upright_model(matrices)
    
    # Design LQR controller
    print("Designing LQR controller...")
    controller = LQRController(upright_matrices.A_c, upright_matrices.B_c, Q, R)
    controller.print_gains()
    
    # Simulate
    simulator = LQRSimulator(upright_matrices.A_c, upright_matrices.B_c, controller)
    results = simulator.simulate(initial_state=np.array([0, 0, 0.4, 0]))
    
    # Plot
    simulator.plot_results(results)
