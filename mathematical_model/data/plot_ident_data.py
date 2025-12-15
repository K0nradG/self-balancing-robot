import pandas as pd
import matplotlib.pyplot as plt

# Wczytanie danych
df = pd.read_csv("merged.csv")

# Konwersja dt na float (na wypadek błędów typu "0na.002")
df["dt"] = pd.to_numeric(df["dt"], errors="coerce")

# Usunięcie wierszy z błędnym dt
df = df.dropna(subset=["dt"])

# Oś czasu – suma kolejnych dt
time = df["dt"].cumsum()

# Utworzenie 5 wykresów pod sobą
fig, axs = plt.subplots(5, 1, sharex=True, figsize=(10, 12))

axs[0].plot(time, df["pwm"])
axs[0].set_ylabel("PWM")
axs[0].grid(True)

axs[1].plot(time, df["angle"])
axs[1].set_ylabel("Kąt [rad]")
axs[1].grid(True)

axs[2].plot(time, df["angle_dt"])
axs[2].set_ylabel("d(Kąt)/dt")
axs[2].grid(True)

axs[3].plot(time, df["pos"])
axs[3].set_ylabel("Położenie")
axs[3].grid(True)

axs[4].plot(time, df["pos_dt"])
axs[4].set_ylabel("d(Położenie)/dt")
axs[4].set_xlabel("Czas [s]")
axs[4].grid(True)

plt.tight_layout()
plt.show()

