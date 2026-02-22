import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
import scipy.linalg as la
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class RobotData:
    """Stores processed robot data"""
    time: np.ndarray
    pos: np.ndarray
    pos_dt: np.ndarray
    angle: np.ndarray
    angle_dt: np.ndarray
    pwm: np.ndarray
    dt: float


@dataclass
class ModelMatrices:
    """Stores identified model matrices"""
    A_d: np.ndarray  # discrete-time system matrix
    B_d: np.ndarray  # discrete-time input matrix
    A_c: np.ndarray  # continuous-time system matrix
    B_c: np.ndarray  # continuous-time input matrix
    
    @property
    def eigenvalues_d(self):
        return np.linalg.eigvals(self.A_d)


class DataProcessor:
    """Handles data loading, filtering and preprocessing"""
    
    def __init__(self, cutoff_freq: float = 15.0):
        self.cutoff_freq = cutoff_freq
        self.dt = None
        self.fs = None
        
    def load_and_preprocess(self, file_path: str, t_start: float, t_end: float) -> RobotData:
        """Load CSV and preprocess data within time window"""
        # Load data
        df = pd.read_csv(file_path)
        df['time'] = df['dt'].cumsum()
        self.dt = df['dt'].mean()
        self.fs = 1.0 / self.dt
        
        # Filter velocities
        df = self._apply_lowpass_filter(df)
        
        # Extract time window
        mask = (df['time'] >= t_start) & (df['time'] <= t_end)
        df_cut = df.loc[mask].copy()
        
        # Remove angle offset (hanging equilibrium)
        angle_off = df_cut['angle'].mean()
        df_cut['angle_c'] = df_cut['angle'] - angle_off
        
        return RobotData(
            time=df_cut['time'].values,
            pos=df_cut['pos'].values,
            pos_dt=df_cut['pos_dt_f'].values,
            angle=df_cut['angle_c'].values,
            angle_dt=df_cut['angle_dt_f'].values,
            pwm=df_cut['pwm'].values,
            dt=self.dt
        )
    
    def _apply_lowpass_filter(self, df: pd.DataFrame) -> pd.DataFrame:
        """Apply 4th order Butterworth lowpass filter"""
        b, a = butter(4, self.cutoff_freq / (0.5 * self.fs), btype='low')
        df['angle_dt_f'] = filtfilt(b, a, df['angle_dt'])
        df['pos_dt_f'] = filtfilt(b, a, df['pos_dt'])
        return df


class ModelIdentifier:
    """Identifies discrete and continuous-time models from data"""
    
    @staticmethod
    def identify(data: RobotData) -> ModelMatrices:
        """Identify model matrices from robot data"""
        # Prepare state and input matrices
        X = np.column_stack([data.pos, data.pos_dt, data.angle, data.angle_dt])
        U = data.pwm.reshape(-1, 1)
        
        # Create shifted matrices for identification
        X_k = X[:-1, :]
        X_k1 = X[1:, :]
        U_k = U[:-1, :]
        
        # Solve least squares: X_k1 = [A_d B_d] * [X_k; U_k]
        Phi = np.column_stack((X_k, U_k))
        theta = np.linalg.lstsq(Phi, X_k1, rcond=None)[0].T
        
        # Extract discrete matrices
        A_d = theta[:, :4]
        B_d = theta[:, 4:].reshape(-1, 1)
        
        # Convert to continuous time
        A_c = la.logm(A_d) / data.dt
        B_c = la.inv(A_d - np.eye(4)) @ A_c @ B_d
        
        return ModelMatrices(
            A_d=A_d,
            B_d=B_d,
            A_c=A_c.real,
            B_c=B_c.real
        )
    
    @staticmethod
    def create_upright_model(matrices: ModelMatrices) -> ModelMatrices:
        """Modify model to be stable around upright position"""
        A_upright = matrices.A_c.copy()
        A_upright[3, 2] = abs(matrices.A_c[3, 2])  # Ensure stability in angle acceleration
        A_upright[1, 2] = -matrices.A_c[1, 2]      # Ensure position coupling
        
        return ModelMatrices(
            A_d=matrices.A_d,
            B_d=matrices.B_d,
            A_c=A_upright,
            B_c=matrices.B_c
        )


class ModelValidator:
    """Validates identified model against real data"""
    
    @staticmethod
    def simulate_discrete(initial_state: np.ndarray, U: np.ndarray, 
                          matrices: ModelMatrices) -> np.ndarray:
        """Simulate discrete-time model"""
        n_steps = len(U)
        X_sim = np.zeros((n_steps + 1, 4))
        X_sim[0] = initial_state
        
        for k in range(n_steps):
            X_sim[k+1] = matrices.A_d @ X_sim[k] + matrices.B_d.flatten() * U[k]
        
        return X_sim
    
    @staticmethod
    def plot_comparison(data: RobotData, X_sim: np.ndarray, 
                        matrices: ModelMatrices, title: str = "Model Validation"):
        """Plot real data vs model prediction"""
        fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
        
        states = [
            ('Position [m]', data.pos, X_sim[:-1, 0], 'forestgreen'),
            ('Velocity [m/s]', data.pos_dt, X_sim[:-1, 1], 'darkorange'),
            ('Angle [rad]', data.angle, X_sim[:-1, 2], 'royalblue'),
            ('Angular Vel [rad/s]', data.angle_dt, X_sim[:-1, 3], 'firebrick')
        ]
        
        for i, (label, real, sim, color) in enumerate(states):
            axes[i].plot(data.time, real, label='Real', color=color, alpha=0.4, lw=2)
            axes[i].plot(data.time, sim, '--', label='Model', color='black', lw=1.5)
            axes[i].set_ylabel(label)
            axes[i].legend(loc='upper right')
            axes[i].grid(True, ls=':')
        
        # PWM plot
        axes[4].step(data.time, data.pwm, color='purple', label='PWM [%]')
        axes[4].set_ylabel('PWM [%]')
        axes[4].set_xlabel('Time [s]')
        axes[4].grid(True, ls=':')
        axes[4].legend(loc='upper right')
        
        plt.suptitle(title, fontsize=16)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()
    
    @staticmethod
    def print_matrices(matrices: ModelMatrices):
        """Pretty print model matrices"""
        np.set_printoptions(suppress=True, precision=4)
        print("\n" + "="*50)
        print("IDENTIFIED MODEL MATRICES")
        print("="*50)
        print("\nDiscrete-Time Matrix A_d (System Dynamics):")
        print(matrices.A_d)
        print("\nDiscrete-Time Matrix B_d (Input Influence):")
        print(matrices.B_d)
        print("-" * 50)
        print("\nContinuous-Time Matrix A_c (Physical Dynamics):")
        print(matrices.A_c)
        print("\nContinuous-Time Matrix B_c (Physical Input):")
        print(matrices.B_c)
        print("="*50 + "\n")


# Main function for standalone execution
def identify_model(file_path: str, t_start: float, t_end: float, 
                   cutoff_freq: float = 15.0, plot: bool = True) -> ModelMatrices:
    """Complete model identification pipeline"""
    
    # Process data
    processor = DataProcessor(cutoff_freq)
    data = processor.load_and_preprocess(file_path, t_start, t_end)
    
    # Identify model
    identifier = ModelIdentifier()
    matrices = identifier.identify(data)
    
    # Validate
    validator = ModelValidator()
    X_sim = validator.simulate_discrete(
        np.array([data.pos[0], data.pos_dt[0], data.angle[0], data.angle_dt[0]]),
        data.pwm, matrices
    )
    
    if plot:
        validator.plot_comparison(data, X_sim, matrices, 
                                 f"Model Validation ({t_start}s-{t_end}s)")
        validator.print_matrices(matrices)
    
    return matrices


# Jeśli uruchamiamy bezpośrednio
if __name__ == "__main__":
    # Przykład użycia
    FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
    T_START, T_END = 180.0, 215.0
    CUTOFF_FREQ = 15
    
    matrices = identify_model(FILE_PATH, T_START, T_END, CUTOFF_FREQ)
