from machine import UART, Pin
import time

uart = UART(1, 115200, tx=Pin(4), rx=Pin(5))
print("Sending AT...")
uart.write(b'AT\r\n')
time.sleep(1)
if uart.any():
    print("Response:", uart.read().decode())
else:
    print("No response from ESP8266.")
