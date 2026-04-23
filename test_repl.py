import serial
import time
ser = serial.Serial('/dev/cu.usbmodem11201', 115200, timeout=1)
ser.write(b'\r\n')
time.sleep(0.5)
print(ser.read(ser.in_waiting).decode('utf-8', errors='replace'))
ser.close()
