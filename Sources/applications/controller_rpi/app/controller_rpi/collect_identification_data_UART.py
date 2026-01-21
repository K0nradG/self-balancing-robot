import serial
import struct
import csv
import time

PORT = "/dev/serial0"
BAUDRATE = 115200
MAGIC = 0xDEADBEEF
# Format: Magic (I), dt (f), angle (f), angle_dt (f), pwm (f), pos (f), pos_dt (f)
FRAME_FORMAT = "<Iffffff"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)

def collect():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
    results = []
    raw_buffer = b""
    
    print(f"Rozpoczęto zbieranie na {PORT}...")

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            
            raw_buffer += chunk

            while len(raw_buffer) >= FRAME_SIZE:
                magic_idx = raw_buffer.find(struct.pack("<I", MAGIC))
                
                if magic_idx != -1:
                    if len(raw_buffer) >= magic_idx + FRAME_SIZE:
                        frame_data = raw_buffer[magic_idx : magic_idx + FRAME_SIZE]
                        decoded = struct.unpack(FRAME_FORMAT, frame_data)
                        
                        results.append(decoded[1:])
                        
                        raw_buffer = raw_buffer[magic_idx + FRAME_SIZE:]
                        
                        if len(results) % 100 == 0:
                            print(f"Odebrano {len(results)} ramek...")
                    else:
                        break
                else:
                    try:
                        text_check = raw_buffer.decode('utf-8', errors='ignore')
                        if "Identification stop" in text_check:
                            print("Wykryto tekstowy komunikat STOP!")
                    except:
                        pass
                    
                    if len(raw_buffer) > 100:
                        raw_buffer = raw_buffer[1:]
                    break

    except KeyboardInterrupt:
        print("Zatrzymano ręcznie.")
    finally:
        if results:
            with open("identification_data.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["dt", "angle", "angle_dt", "pwm", "pos", "pos_dt"])
                writer.writerows(results)
            print(f"Zapisano {len(results)} wierszy do identification_data.csv")
        ser.close()

if __name__ == "__main__":
    collect()