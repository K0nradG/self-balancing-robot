import tkinter as tk
from tkinter import messagebox
import subprocess
import threading
import re
import time

class BluetoothApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bluetooth Manager")
        
        self.devices = {}

        self.scan_button = tk.Button(root, text="Skanuj urządzenia", command=self.scan_devices)
        self.scan_button.pack(pady=10)
        
        self.device_listbox = tk.Listbox(root, width=50)
        self.device_listbox.pack(pady=10)
        
        self.connect_button = tk.Button(root, text="Połącz", command=self.connect_device)
        self.connect_button.pack(pady=10)
        
        self.notify_button = tk.Button(root, text="Włącz powiadomienia", command=self.enable_notifications)
        self.notify_button.pack(pady=10)

    def run_command(self, command):
        """Uruchamia komendę w shellu i zwraca output."""
        try:
            output = subprocess.check_output(command, shell=True, text=True)
            return output
        except subprocess.CalledProcessError:
            return ""

    def scan_devices(self):
        """Skanuje urządzenia Bluetooth i aktualizuje listę w GUI."""
        self.device_listbox.delete(0, tk.END)
        self.devices.clear()

        def scan():
            print("🔍 Rozpoczynam skanowanie...")
            process = subprocess.Popen(["bluetoothctl"], 
                                       stdin=subprocess.PIPE, 
                                       stdout=subprocess.PIPE, 
                                       stderr=subprocess.PIPE, 
                                       text=True, 
                                       bufsize=1, 
                                       universal_newlines=True)

            process.stdin.write("scan on\n")
            process.stdin.flush()

            time.sleep(10)  # Czekamy 10 sekund na wykrycie urządzeń

            process.stdin.write("scan off\n")
            process.stdin.flush()
            
            output, _ = process.communicate(timeout=2)  # Pobieramy cały output
            process.terminate()
            
            print("📡 Otrzymane dane:\n", output)  # Debugowanie: sprawdzamy, co zwraca bluetoothctl
            
            znalezione_urzadzenia = {}

            # Przeszukujemy output w poszukiwaniu urządzeń
            for line in output.split("\n"):
                line = line.strip()
                print("LOG:", line)  # Debugowanie każdej linii
                
                # Nowe regex: obsługujemy przypadek z `[bluetooth]#`
                match = re.search(r"Device ([0-9A-F:]+) (.+)", line)
                if match:
                    mac = match.group(1)
                    name = match.group(2).strip()
                    if name and mac not in znalezione_urzadzenia.values():
                        znalezione_urzadzenia[name] = mac
                        print(f"✅ Znaleziono: {name} ({mac})")

            # Aktualizacja GUI w głównym wątku Tkintera
            self.root.after(0, lambda: self.update_device_list(znalezione_urzadzenia))

        threading.Thread(target=scan, daemon=True).start()

    def update_device_list(self, new_devices):
        """Aktualizuje listę urządzeń w interfejsie graficznym."""
        if not new_devices:
            messagebox.showwarning("Brak urządzeń", "Nie znaleziono żadnych urządzeń Bluetooth.")
            return

        self.devices = new_devices  # Zapisanie nowych urządzeń
        self.device_listbox.delete(0, tk.END)
        for name in self.devices:
            self.device_listbox.insert(tk.END, name)

    def connect_device(self):
        """Łączy się z wybranym urządzeniem."""
        selected = self.device_listbox.curselection()
        if not selected:
            messagebox.showwarning("Błąd", "Wybierz urządzenie!")
            return
        
        device_name = self.device_listbox.get(selected[0])
        mac_address = self.devices.get(device_name)
        if mac_address:
            output = self.run_command(f"bluetoothctl connect {mac_address}")
            messagebox.showinfo("Info", f"Połączono z {device_name}\n{output}")

    def enable_notifications(self):
        """Włącza powiadomienia dla wybranego urządzenia."""
        selected = self.device_listbox.curselection()
        if not selected:
            messagebox.showwarning("Błąd", "Wybierz urządzenie!")
            return
        
        device_name = self.device_listbox.get(selected[0])
        mac_address = self.devices.get(device_name)
        if mac_address:
            service_path = f"/org/bluez/hci0/dev_{mac_address.replace(':', '_')}"
            self.run_command(f"bluetoothctl select-attribute {service_path}/service0010/char0011")
            output = self.run_command("bluetoothctl notify on")
            messagebox.showinfo("Info", f"Powiadomienia włączone dla {device_name}\n{output}")

if __name__ == "__main__":
    root = tk.Tk()
    app = BluetoothApp(root)
    root.mainloop()

