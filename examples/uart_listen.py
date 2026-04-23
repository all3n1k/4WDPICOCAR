from machine import UART, Pin
import time

led = Pin(25, Pin.OUT)
uart = UART(1, 115200, timeout=1000)

led.value(1)
print("Listening for UART data for 10 seconds...")
led.value(0)

start = time.time()
while time.time() - start < 10:
    if uart.any():
        data = uart.read(100)
        if data:
            print("RX:", data.decode())
    led.toggle()
    time.sleep(0.5)

print("Done")