import serial
import struct
import csv
import time

PORT = "/dev/serial0"
BAUDRATE = 1000000
MAGIC = 0xDEADBEEF
FRAME_FORMAT = "<Iffffff"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)

def collect():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
    results = []
    raw_buffer = b""
    should_stop = False
    
    print(f"Started collecting on {PORT} at {BAUDRATE} bps...")

    try:
        while not should_stop:
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
                            print(f"Received {len(results)} frames...")
                    else:
                        break
                else:
                    if b"Identification stop" in raw_buffer:
                        print("\n>>> Identification stopped <<<")
                        should_stop = True
                        break
                    
                    if len(raw_buffer) > 200:
                        raw_buffer = raw_buffer[-50:]
                    break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if results:
            with open("identification_data.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["dt", "angle", "angle_dt", "pwm", "pos", "pos_dt"])
                writer.writerows(results)
            print(f"Saved {len(results)} rows to identification_data.csv")
        else:
            print("No data received.")
        ser.close()

if __name__ == "__main__":
    collect()