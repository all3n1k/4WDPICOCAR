"""
Direct servo test — bypasses all agent code.
Sends S20/S21 commands directly to the Pico WebSocket.

Usage:
    python3 test_servos.py

Expects the Pico to be running its firmware and connected to WiFi.
"""
import json, time, websocket

WS_URL = "ws://192.168.1.217:8765"

print(f"Connecting to {WS_URL}...")
try:
    ws = websocket.create_connection(WS_URL, timeout=5)
except Exception as e:
    print(f"FAILED to connect: {e}")
    print("Is the Pico powered on and connected to WiFi?")
    exit(1)

print("Connected!")

def send(payload):
    msg = json.dumps(payload)
    print(f"  → Sending: {msg}")
    ws.send(msg)
    time.sleep(0.5)

print("\n--- Testing GP20 (Pan) ---")
print("Sweeping GP20 left (angle 0 → -90 in Pico scale)...")
send({"S20": 0})   # 0 - 90 = -90 (full left)
time.sleep(1)
print("Centering GP20...")
send({"S20": 90})  # 90 - 90 = 0 (center)
time.sleep(1)
print("Sweeping GP20 right (angle 180 → +90 in Pico scale)...")
send({"S20": 180}) # 180 - 90 = +90 (full right)
time.sleep(1)
print("Re-centering GP20...")
send({"S20": 90})

print("\n--- Testing GP21 (Tilt) ---")
print("Tilting GP21 up (angle 0)...")
send({"S21": 0})
time.sleep(1)
print("Centering GP21...")
send({"S21": 90})
time.sleep(1)
print("Tilting GP21 down (angle 180)...")
send({"S21": 180})
time.sleep(1)
print("Re-centering GP21...")
send({"S21": 90})

print("\n--- Done. ---")
print("If servos moved: firmware is working, the bug is in the agent/dashboard chain.")
print("If servos did NOT move: the firmware or hardware is the problem.")
ws.close()
