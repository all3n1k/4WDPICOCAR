import json
import time
import threading
import websocket
from dimos_lite.core import Module, StreamIn, StreamOut

VALID_COMMANDS = {"forward", "backward", "left", "right", "stop"}


class PicoHardwareModule(Module):
    def __init__(self, ws_url="ws://192.168.4.1:8765"):
        super().__init__("PicoHW")
        self.ws_url = ws_url
        self._ws = None
        self._connected = False
        self._lock = threading.Lock()

        self.ultrasonic_distance = StreamOut("ultrasonic_distance")
        self.grayscale_line = StreamOut("grayscale_line")
        self.odometry = StreamOut("odometry")
        self.speed_cm_s = StreamOut("speed_cm_s")
        self.imu_data = StreamOut("imu_data")
        self.tof_distance = StreamOut("tof_distance")

        self.cmd_vel = StreamIn("cmd_vel")
        self.cmd_vel.subscribe(self._on_cmd_vel)

    @property
    def connected(self):
        return self._connected

    def start(self):
        print(f"[{self.name}] Connecting to {self.ws_url}")

        def on_message(ws, message):
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                return
            if 'D' in data:
                sweep = data['D']
                if isinstance(sweep, list) and sweep:
                    if isinstance(sweep[0], list):
                        for reading in sweep:
                            self.ultrasonic_distance.publish(reading)
                    else:
                        self.ultrasonic_distance.publish(sweep)
            if 'H' in data:
                self.grayscale_line.publish(data['H'])
            if 'C' in data:
                self.odometry.publish(data['C'])
            if 'B' in data:
                self.speed_cm_s.publish(data['B']) # Added B publish
            if 'I' in data:
                self.imu_data.publish(data['I'])
            if 'T' in data:
                v = data['T']
                self.tof_distance.publish(float(v) if v is not None else None)

        def on_open(ws):
            self._connected = True
            print(f"[{self.name}] Connected")
            ws.send(json.dumps({
                "Name": "dimos_lite",
                "Type": "Mac Brain",
                "Check": "Agent OS",
            }))

        def on_close(ws, status_code, msg):
            self._connected = False
            print(f"[{self.name}] Disconnected (status={status_code})")

        def on_error(ws, error):
            print(f"[{self.name}] WS error: {error}")

        def run_ws():
            while True:
                ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=on_open,
                    on_message=on_message,
                    on_close=on_close,
                    on_error=on_error,
                )
                with self._lock:
                    self._ws = ws
                ws.run_forever()
                self._connected = False
                time.sleep(2)

        threading.Thread(target=run_ws, daemon=True).start()

    def _on_cmd_vel(self, data):
        with self._lock:
            if not self._connected or not self._ws:
                return
            ws = self._ws
        

        # New: Handle dictionary payloads (LEDs, Servo, etc.) directly
        if isinstance(data, dict):
            payload = data
        # Handle legacy 2-tuples (direction, speed)
        elif isinstance(data, (list, tuple)):
            cmd = str(data[0])
            cmd_lower = cmd.lower()
            if cmd_lower in VALID_COMMANDS:
                payload = {"K": cmd_lower, "A": 0 if cmd_lower == "stop" else int(data[1] or 0)}
            elif cmd_lower == "pan":
                payload = {"S20": int(data[1])}
            elif cmd == "tilt":
                payload = {"S21": int(data[1])}
            else:
                # Direct pass-through for special commands like ("L", None)
                payload = {cmd: data[1]}
        else:
            cmd = str(data).lower()
            if cmd in VALID_COMMANDS:
                payload = {"K": cmd, "A": 50 if cmd != "stop" else 0}
            else:
                return

        if ws:
            try:
                ws.send(json.dumps(payload))
            except Exception as e:
                print(f"[{self.name}] Send error: {e}")
