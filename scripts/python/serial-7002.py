import sys
import termios
import tty
import serial

# Ustawienia portu szeregowego
ser = serial.Serial('/dev/ttyACM1', 115200)
print("Połączono z /dev/ttyACM1 (baudrate: 115200)")
print("Naciśnij strzałki (↑, →, ←). Wciśnij ESC aby zakończyć.")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':  # ESC
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return f"\x1b[{ch3}"
        else:
            return ch1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

while True:
    key = get_key()

    if key == '\x1b[A':  # strzałka w górę
        ser.write(b'f')
        print("Wysłano: f")
    elif key == '\x1b[D':  # strzałka w lewo
        ser.write(b'l')
        print("Wysłano: l")
    elif key == '\x1b[C':  # strzałka w prawo
        ser.write(b'r')
        print("Wysłano: r")
    elif key == '\x1b':  # sam ESC — wyjście
        print("Zakończono.")
        break

