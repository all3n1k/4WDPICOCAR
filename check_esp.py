from machine import UART, Pin
import time

uart = UART(1, 115200, timeout=100, timeout_char=10, tx=Pin(8), rx=Pin(9))
print("UART initialized on GP8/GP9. Sending AT...")

# Try to send something and see if it replies
uart.write(b'AT\r\n')
time.sleep(0.5)
resp = uart.read()
print(f"Response: {resp}")

uart.write(b'SET+RESET\r\n')
time.sleep(2)
resp = uart.read()
print(f"Response after reset: {resp}")
