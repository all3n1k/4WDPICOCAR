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


def on_receive(data):
    global last_cmd_ms, timed_out, servo_override
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

        # 2. LED control
        if 'L_BOT' in data:
            car.set_light_bottom_color(tuple(data['L_BOT']))
        if 'L_REAR' in data:
            car.set_light_rear_color(tuple(data['L_REAR']))

        # 3. Motor control
        if 'K' in data and 'A' in data:
            direction = str(data['K'])
            power = int(data['A'])
            if direction == 'right':
                car.set_motor_power(0, -power, -power, power)
            elif direction == 'left':
                car.set_motor_power(-power, 0, power, -power)
            elif direction == 'forward':
                car.set_motor_power(power, power, power, power)
            elif direction == 'backward':
                car.set_motor_power(-power, -power, -power, -power)
            else:
                car.move(direction, power)
    except Exception as e:
        print("pkt err:", e)
        car.stop()

# Wire the packet handler into WS_Server (default is a no-op).
ws.on_receive = on_receive

print("Pico-4WD OS v4.1 started")

while True:
    try:
        ws.loop()

        # Safety: auto-stop if Mac disconnects or stops sending commands
        if time.ticks_diff(time.ticks_ms(), last_cmd_ms) > CMD_TIMEOUT_MS:
            if not timed_out:
                car.stop()
                timed_out = True

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
