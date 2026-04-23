from machine import UART, Pin
import time

uart = UART(1, 115200, timeout=100, timeout_char=10, tx=Pin(8), rx=Pin(9))
print("Reading UART for 3 seconds...")
start = time.time()
while time.time() - start < 3:
    resp = uart.read()
    if resp:
        print(f"Received: {resp}")
    time.sleep(0.1)

print("Sending AT...")
uart.write(b'AT\r\n')
time.sleep(0.5)
print(f"Response: {uart.read()}")
