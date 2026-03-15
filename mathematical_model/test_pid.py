import numpy as np
import matplotlib.pyplot as plt
from model_identification import DataProcessor, ModelIdentifier, identify_model
from dataclasses import dataclass


@dataclass
class PIDConfig:
    """PID controller configuration"""
    kp: float = 40.0
    ki: float = 40.0
    kd: float = 3.0
    output_limit: float = 100.0
    hysteresis: float = 0.0
    dt_min: float = 0.001
    dt_max: float = 0.05
    epsilon: float = 1e-3


class LowPassFilter:
    """Simple first-order low-pass filter"""
    
    def __init__(self, alpha: float = 1.0):
        self.alpha = alpha
        self.state = 0.0
    
    def filter(self, value: float) -> float:
        self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state


class PIDController:
    """PID controller with anti-windup"""
    
    def __init__(self, config: PIDConfig):
        self.config = config
        self.integral = 0.0
        self.prev_error = 0.0
        self.filter = LowPassFilter(alpha=1.0)
    
    def compute(self, setpoint: float, feedback: float, dt: float) -> float:
        """
        Compute PID output
        
        Args:
            setpoint: desired value
            feedback: measured value
            dt: time step
            
        Returns:
            control output
        """
        # Limit dt for numerical stability
        dt = max(min(dt, self.config.dt_max), self.config.dt_min)
        
        # Filter feedback and compute error
        error = setpoint - self.filter.filter(feedback)
        
        # Apply hysteresis
        if abs(error) < self.config.hysteresis:
            self.integral = 0.0
            self.prev_error = error
            return 0.0
        
        # Integral term with anti-windup
        if abs(self.config.ki) > self.config.epsilon:
            self.integral += self.config.ki * error * dt
        else:
            self.integral = 0.0
        
        # Derivative term
        derivative = self.config.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        # Compute output
        output = self.config.kp * error + self.integral + derivative
        saturated_output = np.clip(output, -self.config.output_limit, 
                                   self.config.output_limit)
        
        # Anti-windup: reduce integral if saturated
        if (abs(output - saturated_output) > self.config.epsilon and 
            output * error > 0):
            self.integral -= self.config.ki * error * dt
        
        return saturated_output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.filter.state = 0.0


class PIDSimulator:
    """Simulates closed-loop system with PID controller"""
    
    def __init__(self, A_d: np.ndarray, B_d: np.ndarray):
        """
        Args:
            A_d: discrete-time system matrix
            B_d: discrete-time input matrix
        """
        self.A_d = A_d
        self.B_d = B_d
    
    def simulate(self, initial_state: np.ndarray, pid_config: PIDConfig,
                 T_sim: float = 10.0, dt: float = 0.002) -> dict:
        """
        Simulate discrete-time closed-loop response
        
        Returns:
            dict with 'time', 'states', 'control' arrays
        """
        steps = int(T_sim / dt)
        t = np.linspace(0, T_sim, steps)
        
        X = np.zeros((steps, 4))
        U = np.zeros(steps)
        x = initial_state.copy()
        
        pid = PIDController(pid_config)
        
        for i in range(steps):
            # Store current state
            X[i] = x
            
            # Compute control based on angle
            u = pid.compute(0.0, x[2], dt)
            U[i] = u
            
            # Discrete-time update
            x = self.A_d @ x + self.B_d.flatten() * u
        
        return {
            'time': t,
            'states': X,
            'control': U
        }
    
    def plot_results(self, results: dict, title: str = "PID Simulation"):
        """Plot simulation results"""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
        
        t = results['time']
        X = results['states']
        U = results['control']
        
        # Angle plot
        ax1.plot(t, X[:, 2], label='Angle [rad]', color='blue', lw=2)
        ax1.axhline(0, color='black', lw=1, ls='--')
        ax1.set_title(f"{title}: Angle Recovery")
        ax1.set_ylabel("Angle [rad]")
        ax1.legend()
        ax1.grid(True, ls=':')
        
        # Position plot
        ax2.plot(t, X[:, 0], label='Position [m]', color='green', lw=2)
        ax2.axhline(0, color='black', lw=1, ls='--')
        ax2.set_title("Position Response")
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
    
    @staticmethod
    def print_eigenvalues(A_d: np.ndarray):
        """Print discrete system eigenvalues"""
        eigvals = np.linalg.eigvals(A_d)
        print("\nDiscrete system eigenvalues:")
        for i, eig in enumerate(eigvals):
            print(f"  λ{i} = {eig.real:.4f} + {eig.imag:.4f}j")


# Main execution
if __name__ == "__main__":
    # Configuration
    FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
    T_START, T_END = 180.0, 215.0
    CUTOFF_FREQ = 15
    
    # PID tuning
    pid_config = PIDConfig(
        kp=40.0,
        ki=40.0,
        kd=3.0,
        output_limit=100.0
    )
    
    # Identify model
    print("Identifying model...")
    matrices = identify_model(FILE_PATH, T_START, T_END, CUTOFF_FREQ, plot=False)
    
    # Print eigenvalues
    PIDSimulator.print_eigenvalues(matrices.A_d)
    
    # Simulate PID control
    simulator = PIDSimulator(matrices.A_d, matrices.B_d)
    results = simulator.simulate(
        initial_state=np.array([0, 0, 0.3, 0]),
        pid_config=pid_config,
        T_sim=10.0
    )
    
    # Plot results
    simulator.plot_results(results)
