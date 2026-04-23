from machine import UART, Pin
import time

led = Pin(25, Pin.OUT)

configs = [
    (0, 115200),
    (0, 9600),
    (1, 115200),
    (1, 9600),
]

print("Scanning UARTs...")

for uart_id, baud in configs:
    led.value(1)
    print(f"\n=== UART{uart_id} @ {baud} ===")
    try:
        uart = UART(uart_id, baud, timeout=500)
        time.sleep(0.5)
        
        # Send AT
        uart.write(b"AT\r\n")
        time.sleep(1)
        
        # Listen for response
        response = b""
        start = time.time()
        while time.time() - start < 2:
            if uart.any():
                response += uart.read(100)
            time.sleep(0.1)
        
        if response:
            print("Got:", response.decode())
        else:
            print("No response")
            
    except Exception as e:
        print("Error:", e)
    
    led.value(0)
    time.sleep(0.5)

print("\nDone")