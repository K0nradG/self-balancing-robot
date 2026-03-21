# Copyright 2026 Filip Dymczyk and Konrad Grucel


import numpy as np
import matplotlib.pyplot as plt

voltage_intervals = [8400, 7900, 7400, 7000, 6600, 6000]  # mV
charge_levels = [100, 80, 60, 30, 10, 0]  # %

voltage_values = []
charge_values = []

for i in range(len(voltage_intervals) - 1):
    v_start, v_end = voltage_intervals[i], voltage_intervals[i + 1]
    c_start, c_end = charge_levels[i], charge_levels[i + 1]
    
    v_range = np.linspace(v_start, v_end, num=20)
    c_range = np.linspace(c_start, c_end, num=20)
    
    voltage_values.extend(v_range)
    charge_values.extend(c_range)

plt.figure(figsize=(8,5))
plt.plot(voltage_values, charge_values, 'b-', linewidth=2, label='Battery Discharge Curve')
plt.xlabel("Voltage (mV)")
plt.ylabel("Charge Level (%)")
plt.title("LiPo 7.4V Battery Discharge Curve")
plt.legend()
plt.grid()
plt.show()

