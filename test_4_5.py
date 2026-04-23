from machine import UART, Pin
import time

try:
    uart = UART(1, 115200, timeout=50, timeout_char=10, tx=Pin(4), rx=Pin(5))
    uart.write(b'SET+RESET\n')
    time.sleep(1.0)
    resp = uart.read()
    if resp:
        print(f"GP4/GP5 RESPONDED: {resp}")
    else:
        print(f"GP4/GP5 silent")
except Exception as e:
    print(f"GP4/GP5 error: {e}")
