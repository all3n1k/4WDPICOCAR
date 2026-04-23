from machine import UART, Pin
import time

uart = UART(1, 115200, tx=Pin(8), rx=Pin(9))
print("Sending AT to GP8/GP9...")
uart.write(b'AT\r\n')
time.sleep(1)
if uart.any():
    print("Response:", uart.read().decode())
else:
    print("No response from ESP8266.")
