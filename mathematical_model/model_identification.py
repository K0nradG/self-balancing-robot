import pandas as pd
import numpy as np
import scipy.linalg as la
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class ModelMatrices:
    """Struktura przechowująca macierze modelu."""
    A_c: np.ndarray; B_c: np.ndarray
    A_d: np.ndarray; B_d: np.ndarray

class TrajectoryIdentifier:
    def __init__(self, deadband=10.0):
        self.deadband = deadband 

    def _apply_deadband(self, pwm):
        """Kompensacja strefy nieczułości silników."""
        return np.sign(pwm) * np.maximum(0, np.abs(pwm) - self.deadband)

    def system_ode(self, x, t, u_func, p):
        """
        Model ciągły stanu: x = [pos, dpos, angle, dangle]
        p = [a22, a23, a24, b2, a42, a43, a44, b4]
        """
        u = u_func(t)
        d_pos = x[1]
        # Równanie przyspieszenia liniowego
        dd_pos = p[0]*x[1] + p[1]*x[2] + p[2]*x[3] + p[3]*u
        d_angle = x[3]
        # Równanie przyspieszenia kątowego
        dd_angle = p[4]*x[1] + p[5]*x[2] + p[6]*x[3] + p[7]*u
        return [d_pos, dd_pos, d_angle, dd_angle]

    def loss_function(self, p, time, pwm, real_pos, real_angle):
        """Funkcja celu: różnica między symulacją a rzeczywistością."""
        u_func = interp1d(time, self._apply_deadband(pwm), fill_value="extrapolate")
        x0 = [real_pos[0], 0, real_angle[0], 0]
        
        try:
            sol = odeint(self.system_ode, x0, time, args=(u_func, p))
            # Wagi błędu: angle (100) vs pos (1)
            err = np.mean((real_pos - sol[:,0])**2) + 100 * np.mean((real_angle - sol[:,2])**2)
            
            # Kary fizyczne (wymuszanie stabilności zwisu)
            if p[0] > 0: err += p[0]*5000 
            if p[6] > 0: err += p[6]*5000
            return err
        except:
            return 1e12

    def identify(self, df, t_start, t_end):
        """Główna pętla identyfikacji parametrów."""
        mask = (df['time'] >= t_start) & (df['time'] <= t_end)
        df_cut = df.loc[mask].copy()
        dt = df_cut['dt'].mean()
        
        # Precyzyjne zerowanie kąta (pierwsze 0.2s)
        angle_offset = df_cut['angle'].iloc[:max(1, int(0.2/dt))].mean()
        angle_c = df_cut['angle'].values - angle_offset
        pos_c = df_cut['pos'].values - df_cut['pos'].iloc[0]
        time = df_cut['time'].values - df_cut['time'].iloc[0]
        pwm = df_cut['pwm'].values

        # Początkowe parametry [a22, a23, a24, b2, a42, a43, a44, b4]
        p0 = [-10.0, -2.0, 0.0, 0.2, 50.0, -100.0, -5.0, -0.2]
        
        print(f"Rozpoczynam optymalizację (okno: {t_end-t_start:.1f}s)...")
        res = minimize(self.loss_function, p0, args=(time, pwm, pos_c, angle_c), 
                       method='Nelder-Mead', options={'maxiter': 2000})
        
        p = res.x
        A_c = np.array([[0, 1, 0, 0], [0, p[0], p[1], p[2]], [0, 0, 0, 1], [0, p[4], p[5], p[6]]])
        B_c = np.array([[0], [p[3]], [0], [p[7]]])
        
        # Dyskretyzacja (Zero-Order Hold)
        sys_d = la.expm(np.block([[A_c, B_c], [np.zeros((1, 5))]]) * dt)
        return ModelMatrices(A_c, B_c, sys_d[:4, :4], sys_d[:4, 4:]), (time, pwm, pos_c, angle_c, p)

def plot_final_validation(debug_data, deadband):
    """Generuje wykresy porównawcze po identyfikacji."""
    time, pwm, real_pos, real_angle, p = debug_data
    u_func = interp1d(time, np.sign(pwm) * np.maximum(0, np.abs(pwm) - deadband), fill_value="extrapolate")
    x0 = [real_pos[0], 0, real_angle[0], 0]
    
    ident = TrajectoryIdentifier(deadband=deadband)
    sol = odeint(ident.system_ode, x0, time, args=(u_func, p))

    fig, ax = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    ax[0].plot(time, real_pos, label='Dane (Enkoder)', alpha=0.7)
    ax[0].plot(time, sol[:,0], '--', label='Model (Symulacja)', color='black')
    ax[0].set_ylabel('Pozycja [m]')
    
    ax[1].plot(time, real_angle, label='Dane (IMU)', color='green', alpha=0.7)
    ax[1].plot(time, sol[:,2], '--', label='Model (Symulacja)', color='black')
    ax[1].set_ylabel('Kąt [rad]')
    
    ax[2].step(time, pwm, color='purple', label='Sygnał PWM')
    ax[2].set_ylabel('PWM [%]')
    ax[2].set_xlabel('Czas [s]')
    
    for a in ax: a.legend(); a.grid(True, ls=':')
    plt.suptitle('Walidacja Modelu: Trajectory Matching', fontsize=14)
    plt.tight_layout()
    plt.show()

def create_upright_model(hanging_matrices, dt):
    """Konwersja: zwis -> balans (odwrócenie znaku grawitacji)."""
    A_up = hanging_matrices.A_c.copy()
    A_up[1, 2] = -A_up[1, 2]
    A_up[3, 2] = -A_up[3, 2]
    
    sys_d = la.expm(np.block([[A_up, hanging_matrices.B_c], [np.zeros((1, 5))]]) * dt)
    return ModelMatrices(A_up, hanging_matrices.B_c, sys_d[:4, :4], sys_d[:4, 4:])

def run_full_process(csv_path, t_start, t_end, db=12.0):
    df = pd.read_csv(csv_path)
    df['time'] = df['dt'].cumsum()
    dt = df['dt'].mean()

    ident = TrajectoryIdentifier(deadband=db)
    hanging_model, debug_data = ident.identify(df, t_start, t_end)
    
    # Wywołanie lokalnej funkcji rysującej
    plot_final_validation(debug_data, db)
    
    upright_model = create_upright_model(hanging_model, dt)
    
    np.set_printoptions(suppress=True, precision=4)
    print("\n" + "="*30)
    print("IDENTYFIKACJA ZAKOŃCZONA")
    print("="*30)
    print("Macierz A_c (Stojący):\n", upright_model.A_c)
    print("Macierz B_c (Stojący):\n", upright_model.B_c)
    print("="*30)
    
    return upright_model

if __name__ == "__main__":
    # PODSTAW SWOJĄ ŚCIEŻKĘ
    FILE_PATH = 'data/identification_signal_1/robot_identification_side_backward_2.csv'
    # Wybierz okno czasowe, gdzie robot wykonuje wyraźne ruchy
    run_full_process(FILE_PATH, 100.0, 145.0, db=5.0)
