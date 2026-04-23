from machine import UART, Pin
import time

led = Pin(25, Pin.OUT)
uart = UART(1, 115200)

led.value(1)
print("=== ESP UART Test ===")

# Try sending commands the Sunfounder way
cmds = [
    b"",  # empty line first
    b"AT\r\n",
    b"AT+GMR\r\n", 
    b"AT+RESTORE\r\n",
]

for cmd in cmds:
    print(f"Sending: {cmd}")
    uart.write(cmd)
    time.sleep(1)
    if uart.any():
        while uart.any():
            data = uart.read(64)
            if data:
                print(f"  Got: {data}")
    time.sleep(0.5)

print("=== Test Complete ===")
led.value(0)