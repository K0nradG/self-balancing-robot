import serial
import os

def parse_serial_data(port, log_file_name, baudrate=115200, timeout=1):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_dir = os.path.join(script_dir, 'data')

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    log_file_path = os.path.join(log_dir, log_file_name)

    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"Listening on {port} at {baudrate} baud...")
            
            data_receiving = False
            with open(log_file_path, "w") as f:
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    if "[INF] MODEL: model data sending start" in line:
                        print("Start signal received, collecting data...")
                        data_receiving = True
                        continue

                    if "[INF] MODEL: model data sending finished" in line:
                        print("Stop signal received, ending collection...")
                        break
                    
                    if data_receiving:
                        print(line)
                        f.write(line + "\n")

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    port = "COM4"
    log_file = "minicom.txt" 
    parse_serial_data(port, log_file)

    print(f"Data logged to data/{log_file}.")