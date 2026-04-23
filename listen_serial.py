import serial
import sys
import time

try:
    ser = serial.Serial('/dev/cu.usbmodem11201', 115200, timeout=1)
    print("Listening to Pico...")
    start = time.time()
    while time.time() - start < 15: # Listen for 15 seconds
        if ser.in_waiting > 0:
            sys.stdout.write(ser.read(ser.in_waiting).decode('utf-8', errors='replace'))
            sys.stdout.flush()
        time.sleep(0.1)
    ser.close()
except Exception as e:
    print(f"Error: {e}")
