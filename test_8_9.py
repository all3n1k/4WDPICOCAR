from machine import UART, Pin
import time

try:
    uart = UART(1, 115200, timeout=50, timeout_char=10, tx=Pin(8), rx=Pin(9))
    uart.write(b'SET+RESET\n')
    time.sleep(1.0)
    resp = uart.read()
    if resp:
        print(f"GP8/GP9 RESPONDED: {resp}")
    else:
        print(f"GP8/GP9 silent")
except Exception as e:
    print(f"GP8/GP9 error: {e}")
