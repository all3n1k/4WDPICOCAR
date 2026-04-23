from machine import UART, Pin
import time

pairs = [(0, 1), (4, 5), (8, 9), (12, 13)]
bauds = [115200, 9600, 74880]

for tx, rx in pairs:
    for baud in bauds:
        try:
            uart = UART(1, baud, tx=Pin(tx), rx=Pin(rx), timeout=100)
            uart.write(b'AT\r\n')
            time.sleep(0.5)
            if uart.any():
                print(f"Found on tx={tx}, rx={rx} @ {baud}:", uart.read())
        except Exception as e:
            pass
        try:
            uart = UART(0, baud, tx=Pin(tx), rx=Pin(rx), timeout=100)
            uart.write(b'AT\r\n')
            time.sleep(0.5)
            if uart.any():
                print(f"Found on tx={tx}, rx={rx} @ {baud}:", uart.read())
        except Exception as e:
            pass
print("Sweep complete.")
