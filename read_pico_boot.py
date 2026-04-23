import serial
import time
import glob

ports = glob.glob('/dev/tty.usbmodem*')
if not ports:
    print("No Pico found")
    exit(1)

port = ports[0]
print(f"Connecting to {port}")
ser = serial.Serial(port, 115200, timeout=1)

# Send Ctrl-C to interrupt current loop, then Ctrl-D for soft reboot
ser.write(b'\x03\x04')

start = time.time()
while time.time() - start < 10:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(line)
        if "WebServer started" in line:
            break
ser.close()
