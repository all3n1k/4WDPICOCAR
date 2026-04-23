from machine import UART, Pin
import time
import sys

led = Pin(25, Pin.OUT)
uart = UART(1, 115200, timeout=500, timeout_char=20)

def blink(n, delay=0.2):
    for _ in range(n):
        led.value(1)
        time.sleep(delay)
        led.value(0)
        time.sleep(delay)

def send_at(cmd):
    print("Sending: " + cmd)
    uart.write(cmd + "\r\n")
    time.sleep(1)
    response = ""
    while uart.any():
        b = uart.read(100)
        if b:
            response += b.decode()
    if response:
        print("Response: " + response.strip())
    else:
        print("No response")
    return response

print("=== Starting ESP config ===")
blink(3, 0.1)
time.sleep(1)

print("Waiting for ESP to boot...")
time.sleep(2)

print("\n1. Checking AT...")
blink(2, 0.1)
resp = send_at("AT")
if "OK" not in resp:
    print("ESP not responding! LED blink 5x")
    blink(5, 0.3)
    sys.exit(1)

blink(2, 0.1)
print("\n2. Restoring factory defaults...")
send_at("AT+RESTORE")
time.sleep(2)

print("\n3. Setting AP mode...")
blink(1, 0.1)
send_at("AT+CWMODE=2")

print("\n4. Setting SSID and password...")
blink(1, 0.1)
send_at('AT+CWSAP="TrinitysBitch","Matrix101303",5,4')

print("\n5. Rebooting ESP...")
blink(1, 0.1)
send_at("AT+RST")

time.sleep(3)

print("\n=== Done! LED will blink 10x ===")
blink(10, 0.1)