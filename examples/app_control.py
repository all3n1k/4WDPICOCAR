from ws import WS_Server
from machine import I2C, Pin
import json
import time
import pico_4wd as car

SERVO_OFFSET = 0
MIN_ANGLE = -75
MAX_ANGLE = 90
ANGLE_STEP = 5
TICK_MS = 5        # 200Hz loop — halved for lower command latency
READ_EVERY_N = 4   # sonar every 4 ticks = 50 reads/sec (same as before)
CMD_TIMEOUT_MS = 2000

car.stop()
car.servo.set_angle(SERVO_OFFSET)

# Battery voltage: GP29/ADC3 reads Pico's own VSYS rail (~3.3V regulated),
# NOT the robot battery. No battery ADC is wired on this PCB.
# Stub returns 0.0 so the dashboard shows '---' until a real divider is wired.
def read_battery_v(): return 0.0

# ── MPU6050 IMU ──────────────────────────────────────────────
HAS_IMU = False
imu = None
try:
    from mpu6050 import MPU6050
    i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
    devs = i2c.scan()
    if 0x68 in devs or 0x69 in devs:
        addr = 0x68 if 0x68 in devs else 0x69
        imu = MPU6050(i2c, addr=addr)
        print("MPU6050 at 0x%02x — calibrating (hold still)..." % addr)
        imu.calibrate()
        print("MPU6050 ready")
        HAS_IMU = True
    else:
        print("MPU6050 not on I2C bus (found: %s)" % [hex(d) for d in devs])
except Exception as e:
    print("MPU6050 skip: %s" % e)

NAME = 'Pico4WD'
WIFI_MODE = "sta"
SSID = "Stationite"
PASSWORD = "Matrix101303"

time.sleep(3)
ws = WS_Server(name=NAME, mode=WIFI_MODE, ssid=SSID, password=PASSWORD)
ws.start()

current_angle = 0
sweep_direction = 1
servo_override = None  # None = sweep, int = fixed angle
last_cmd_ms = time.ticks_ms()
timed_out = False
led_prev = None
current_radar_sweep = []
tick_count = 0

# ── Turn-signal state ──────────────────────────────────────────
# Derived from incoming K commands in on_receive, ticked in the main loop.
# Fast blink when turning left, slow blink when turning right
# (matches USER_GUIDE v5.5: "Fast=Left, Slow=Right").
turn_signal = None            # None | 'left' | 'right'
turn_signal_prev = None
turn_signal_phase = False
turn_signal_last_ms = 0
last_bot_color = [0, 0, 0]    # most recent L_BOT from Mac (default rest colour)
TURN_SIGNAL_AMBER = (255, 90, 0)
TURN_SIGNAL_LEFT_MS = 160     # ~3 Hz
TURN_SIGNAL_RIGHT_MS = 320    # ~1.5 Hz


def update_turn_signal(now_ms):
    global turn_signal_prev, turn_signal_phase, turn_signal_last_ms

    if turn_signal != turn_signal_prev:
        # Direction changed — reset phase and repaint both halves.
        turn_signal_prev = turn_signal
        turn_signal_phase = False
        turn_signal_last_ms = now_ms
        if turn_signal is None:
            car.set_light_bottom_color(last_bot_color)
            return

    if turn_signal is None:
        return

    period = TURN_SIGNAL_LEFT_MS if turn_signal == 'left' else TURN_SIGNAL_RIGHT_MS
    if time.ticks_diff(now_ms, turn_signal_last_ms) < period:
        return

    turn_signal_last_ms = now_ms
    turn_signal_phase = not turn_signal_phase
    on_color = list(TURN_SIGNAL_AMBER) if turn_signal_phase else list(last_bot_color)
    if turn_signal == 'left':
        car.set_light_bottom_left_color(on_color)
        car.set_light_bottom_right_color(last_bot_color)
    else:
        car.set_light_bottom_right_color(on_color)
        car.set_light_bottom_left_color(last_bot_color)


def on_receive(data):
    global last_cmd_ms, timed_out, servo_override, turn_signal, last_bot_color
    last_cmd_ms = time.ticks_ms()
    timed_out = False

    try:
        # Debug: show what the brain is sending
        if 'L_BOT' in data or 'K' in data:
            print("Pico Rcv:", data)

        # 1. Servo control
        if 'L' in data:
            val = data['L']
            servo_override = None if val is None else int(val)

        # 2. LED control. L_BOT is the steady underglow colour; we cache it so
        #    turn-signal blinks can fall back to it on the 'off' phase.
        if 'L_BOT' in data:
            last_bot_color = list(data['L_BOT'])
            if turn_signal is None:
                car.set_light_bottom_color(last_bot_color)
        if 'L_REAR' in data:
            car.set_light_rear_color(tuple(data['L_REAR']))

        # 3. Motor control
        if 'K' in data and 'A' in data:
            direction = str(data['K'])
            power = int(data['A'])
            if direction == 'right':
                car.set_motor_power(0, -power, -power, power)
                turn_signal = 'right' if power > 0 else None
            elif direction == 'left':
                car.set_motor_power(-power, 0, power, -power)
                turn_signal = 'left' if power > 0 else None
            elif direction == 'forward':
                car.set_motor_power(power, power, power, power)
                turn_signal = None
            elif direction == 'backward':
                car.set_motor_power(-power, -power, -power, -power)
                turn_signal = None
            else:
                car.move(direction, power)
                turn_signal = None
    except Exception as e:
        print("pkt err:", e)
        car.stop()
        turn_signal = None

# Wire the packet handler into WS_Server (default is a no-op).
ws.on_receive = on_receive

print("Pico-4WD OS v4.1 started")

while True:
    try:
        ws.loop()

        now_ms = time.ticks_ms()

        # Safety: auto-stop if Mac disconnects or stops sending commands
        if time.ticks_diff(now_ms, last_cmd_ms) > CMD_TIMEOUT_MS:
            if not timed_out:
                car.stop()
                turn_signal = None
                timed_out = True

        # Turn-signal LEDs (non-blocking)
        update_turn_signal(now_ms)

        # Servo sweep or fixed angle
        target_angle = current_angle if servo_override is None else servo_override
        car.servo.set_angle(target_angle + SERVO_OFFSET)

        # Sonar reading
        tick_count += 1
        if tick_count >= READ_EVERY_N:
            tick_count = 0
            dist = car.sonar.get_distance()
            if 0 < dist < 300:
                ang = target_angle
                found = False
                for i, entry in enumerate(current_radar_sweep):
                    if entry[0] == ang:
                        current_radar_sweep[i][1] = dist
                        found = True
                        break
                if not found:
                    current_radar_sweep.append([ang, dist])

        # Advance sweep angle only if not overridden
        if servo_override is None:
            current_angle += ANGLE_STEP * sweep_direction
            if current_angle >= MAX_ANGLE:
                current_angle = MAX_ANGLE
                sweep_direction = -1
            elif current_angle <= MIN_ANGLE:
                current_angle = MIN_ANGLE
                sweep_direction = 1
                current_radar_sweep = []

        # IMU update every tick (~100Hz)
        if HAS_IMU:
            try:
                imu.update()
            except Exception as e:
                print("imu err: %s" % e)

        # Telemetry every N ticks
        if tick_count % 5 == 0:
            try:
                ws.send_dict['B'] = car.speed()
                ws.send_dict['C'] = car.speed.mileage
                ws.send_dict['D'] = current_radar_sweep
                ws.send_dict['H'] = car.get_grayscale_values()
                ws.send_dict['V'] = read_battery_v()
                if HAS_IMU:
                    ws.send_dict['I'] = imu.get_telemetry()
            except Exception as e:
                print("telem err: %s" % e)

        time.sleep_ms(TICK_MS)
    except Exception as e:
        print("loop err: %s" % e)
        car.move("stop")
        time.sleep_ms(100)
