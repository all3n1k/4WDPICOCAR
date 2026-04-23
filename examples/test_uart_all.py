from machine import UART, Pin
import time

led = Pin(25, Pin.OUT)
uart = UART(1, 115200, timeout=2000)

# Try different baud rates
bauds = [115200, 9600, 57600, 38400]

for baud in bauds:
    uart.init(baud=baud, timeout=1000)
    led.value(1)
    print(f"Testing baud {baud}...")
    
    uart.write(b"AT\r\n")
    time.sleep(1)
    
    if uart.any():
        data = uart.read(64)
        if data:
            print(f"  GOT DATA: {data}")
            led.value(0)
            break
    else:
        print(f"  No response at {baud}")
        led.value(0)
        time.sleep(0.5)
else:
    print("\nNo UART communication working!")