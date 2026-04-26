from ws import WS_Server
from machine import I2C, SoftI2C, Pin
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

# SoftI2C is used for both sensors. Hardware I2C peripheral on this MicroPython
# build (v1.28.0) fails clock-stretched register reads with EIO on every freq;
# SoftI2C handles stretching cleanly. Cost: ~1ms/tick total — sonar already
# blocks for 10-25ms per read, so SoftI2C is not the bottleneck.

# ── MPU6050 IMU on GP0=SDA, GP1=SCL ──────────────────────────────
HAS_IMU = False
imu = None
imu_i2c = None
try:
    from mpu6050 import MPU6050
    imu_i2c = SoftI2C(sda=Pin(0, Pin.IN, Pin.PULL_UP),
                      scl=Pin(1, Pin.IN, Pin.PULL_UP), freq=100000)
    devs = imu_i2c.scan()
    if 0x68 in devs or 0x69 in devs:
        addr = 0x68 if 0x68 in devs else 0x69
        imu = MPU6050(imu_i2c, addr=addr)
        print("MPU6050 at 0x%02x on GP0/GP1 — calibrating (hold still)..." % addr)
        imu.calibrate()
        print("MPU6050 ready")
        HAS_IMU = True
    else:
        print("MPU6050 not on GP0/GP1 (found: %s)" % [hex(d) for d in devs])
except Exception as e:
    print("MPU6050 skip: %s" % e)

# ── VL53L0X TOF Laser on GP2=SDA, GP3=SCL ────────────────────────
HAS_TOF = False
tof = None
tof_i2c = None
try:
    from vl53l0x import VL53L0X
    tof_i2c = SoftI2C(sda=Pin(2, Pin.IN, Pin.PULL_UP),
                      scl=Pin(3, Pin.IN, Pin.PULL_UP), freq=100000)
    devs_tof = tof_i2c.scan()
    if 0x29 in devs_tof:
        tof = VL53L0X(tof_i2c)
        # Continuous mode: chip ranges back-to-back internally; read() polls
        # the latest cached result in ~5-10ms instead of triggering+waiting 37ms.
        tof.start(period=0)
        print("VL53L0X TOF Laser ready on GP2/GP3 (continuous mode)")
        HAS_TOF = True
    else:
        print("VL53L0X not on GP2/GP3 (found: %s)" % [hex(d) for d in devs_tof])
except Exception as e:
    print("VL53L0X skip: %s" % e)

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

# Pan/Tilt head — updated by on_receive, applied every main-loop tick
pan_angle = 0    # target angle -90..90, maps to GP20
tilt_angle = 0   # target angle -90..90, maps to GP21
pan_current = 0  # actual current angle (interpolated)
tilt_current = 0 # actual current angle (interpolated)
# Interpolation: 1° per 5ms tick = 200°/s. Roughly 4× the 50°/s the dashboard
# arrow keys drive at, so we always catch up between sends but without the
# whip-crack feel of the old 400°/s.
SERVO_INTERP_STEP = 1
# Deadband on incoming targets — small jitter from the brain or the WS pipe
# would otherwise twitch the servo every tick.
SERVO_DEADBAND = 2

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
    global last_cmd_ms, timed_out, servo_override, turn_signal, last_bot_color, pan_angle, tilt_angle
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
        
        # 4. Pan/Tilt Head Servos — store angle, main loop applies it.
        # Deadband ignores micro-changes (LLM tune_parameters jitter, WS noise,
        # arrow-key auto-repeat) so the servo doesn't twitch on every packet.
        if 'S20' in data:
            new_pan = max(-90, min(90, int(data['S20']) - 90))
            if abs(new_pan - pan_angle) >= SERVO_DEADBAND:
                pan_angle = new_pan
        if 'S21' in data:
            new_tilt = max(-90, min(90, int(data['S21']) - 90))
            if abs(new_tilt - tilt_angle) >= SERVO_DEADBAND:
                tilt_angle = new_tilt
        
        # 5. Dynamic Radar Config (for mapping)
        if 'radar' in data:
            cmd = data['radar']
            if cmd == 'config':
                if 'step' in data: global ANGLE_STEP; ANGLE_STEP = int(data['step'])
                if 'poll' in data: global READ_EVERY_N; READ_EVERY_N = int(data['poll'])
                print("Radar config: step=%d poll=%d" % (ANGLE_STEP, READ_EVERY_N))
            elif cmd == 'stop':
                servo_override = current_angle
            elif cmd == 'sweep':
                servo_override = None
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

        # Radar servo sweep or fixed angle
        target_angle = current_angle if servo_override is None else servo_override
        car.servo.set_angle(target_angle + SERVO_OFFSET)

        # Pan/Tilt head — interpolate toward target each tick for smooth motion.
        # Only push a fresh PWM update when the current angle actually changed;
        # writing the same angle 200×/s caused observable micro-twitch on the
        # SG90s.
        if pan_current != pan_angle:
            if pan_current < pan_angle:
                pan_current = min(pan_current + SERVO_INTERP_STEP, pan_angle)
            else:
                pan_current = max(pan_current - SERVO_INTERP_STEP, pan_angle)
            car.servo20.set_angle(pan_current)
        if tilt_current != tilt_angle:
            if tilt_current < tilt_angle:
                tilt_current = min(tilt_current + SERVO_INTERP_STEP, tilt_angle)
            else:
                tilt_current = max(tilt_current - SERVO_INTERP_STEP, tilt_angle)
            car.servo21.set_angle(tilt_current)

        # Distance sensing (Laser + Sonar)
        tick_count += 1
        if tick_count >= READ_EVERY_N:
            tick_count = 0
            dist_sonar = car.sonar.get_distance()
            dist_tof = tof.read() if HAS_TOF else None
            
            # Use laser if valid (up to 200cm), fallback to sonar
            dist = dist_sonar
            if dist_tof is not None and dist_tof < 200:
                dist = dist_tof
            
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
            
            # Add raw TOF to telemetry for dashboard debug
            if HAS_TOF: ws.send_dict['T'] = dist_tof

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

        # Telemetry every 50ms (TICK_MS=5, so every 10 ticks)
        if tick_count % 10 == 0:
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
