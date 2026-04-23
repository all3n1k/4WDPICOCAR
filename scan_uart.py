from machine import UART, Pin
import time

def test_uart(tx_pin, rx_pin):
    try:
        uart = UART(1, 115200, timeout=50, timeout_char=10, tx=Pin(tx_pin), rx=Pin(rx_pin))
        uart.write(b'SET+RESET\n')
        time.sleep(1.0)
        resp = uart.read()
        if resp:
            print(f"GP{tx_pin}/GP{rx_pin} RESPONDED: {resp}")
        else:
            print(f"GP{tx_pin}/GP{rx_pin} silent")
    except Exception as e:
        print(f"GP{tx_pin}/GP{rx_pin} error: {e}")

pairs = [(0,1), (4,5), (8,9), (12,13), (16,17)]
for tx, rx in pairs:
    test_uart(tx, rx)
