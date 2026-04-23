from machine import UART, Pin
import time

led = Pin(25, Pin.OUT)
uart = UART(1, 115200, timeout=2000)

led.value(1)
print("Debug: Testing UART to ESP")
led.value(0)

# Try to communicate with ESP
for i in range(5):
    led.toggle()
    print(f"\nAttempt {i+1}: Sending AT...")
    uart.write(b"AT\r\n")
    time.sleep(1)
    
    if uart.any():
        data = uart.read(100)
        print(f"Got: {data}")
    else:
        print("No response")
    
    time.sleep(1)

print("\nDone testing")