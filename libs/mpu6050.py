"""
MPU6050 MicroPython driver for heading estimation via gyroscope integration.

Wiring: SDA → GP0, SCL → GP1, VCC → 3.3V, GND → GND
Default I2C address: 0x68 (AD0 pin low)

Usage:
    from machine import I2C, Pin
    from mpu6050 import MPU6050

    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    imu = MPU6050(i2c)
    imu.calibrate()       # hold robot still during this

    while True:
        imu.update()      # call every loop tick
        print(imu.heading, imu.gyro_z, imu.accel)
"""

import struct
import time

_ADDR = 0x68
_PWR_MGMT_1 = 0x6B
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_DLPF_CONFIG = 0x1A
_ACCEL_XOUT = 0x3B
_GYRO_XOUT = 0x43

# Gyro full-scale range. ±1000°/s. Earlier we used ±500°/s but the 4WD
# chassis at full throttle on a 3-wheel powered turn produces transient
# rotation rates of 800-1200°/s — those saturated, the gyro flatlined at
# 500, and integration silently captured only ~55% of the rotation. After
# ~6-7 turns the accumulated under-count reached 180° of phantom drift,
# making the heading reference frame appear to "rotate." Symptom-confirmed
# fix. Resolution halves (32.8 LSB/°/s vs 65.5) but is still 40× tighter
# than the 0.75°/s deadzone, so the deadzone-and-auto-bias still kill drift.
_GYRO_FS_SEL = 0x10      # 0x00=±250 0x08=±500 0x10=±1000 0x18=±2000
_GYRO_SCALE = 32.8       # LSB per °/s for ±1000°/s range
# ±2g → 16384 LSB per g
_ACCEL_SCALE = 16384.0


class MPU6050:

    def __init__(self, i2c, addr=_ADDR):
        self.i2c = i2c
        self.addr = addr
        self._bias = [0.0, 0.0, 0.0]
        self._heading = 0.0
        self._last_us = time.ticks_us()
        self.gyro_z = 0.0
        self.accel = (0.0, 0.0, 0.0)
        # Stationary-detection counter for online bias auto-correction
        # (zero-velocity-update). When the robot has been measurably still
        # for ~500ms, we slowly nudge _bias[2] toward the current mean so
        # temperature/age drift doesn't quietly poison the heading.
        self._still_count = 0

        self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, b'\x00')
        time.sleep_ms(100)
        self.i2c.writeto_mem(self.addr, _GYRO_CONFIG, bytes([_GYRO_FS_SEL]))
        self.i2c.writeto_mem(self.addr, _ACCEL_CONFIG, b'\x00')
        # DLPF ~44Hz bandwidth — smooths noise without adding much lag
        self.i2c.writeto_mem(self.addr, _DLPF_CONFIG, b'\x03')

    def calibrate(self, samples=500):
        """Average gyro readings while stationary to find bias offset.
        500 samples × 5ms = ~2.5s of stillness. The robot must NOT move
        during this window — any motion gets averaged into the bias and
        produces a permanent heading drift. Online auto-correction in
        update() nudges this back toward truth whenever the robot sits
        still for ~500ms, so a slightly imperfect cold-start calibration
        will recover within seconds of normal operation."""
        sx, sy, sz = 0, 0, 0
        for _ in range(samples):
            raw = self.i2c.readfrom_mem(self.addr, _GYRO_XOUT, 6)
            sx += struct.unpack('>h', raw[0:2])[0]
            sy += struct.unpack('>h', raw[2:4])[0]
            sz += struct.unpack('>h', raw[4:6])[0]
            time.sleep_ms(5)
        self._bias = [sx / samples, sy / samples, sz / samples]
        self._heading = 0.0
        self._last_us = time.ticks_us()
        self._still_count = 0

    def update(self):
        """Read sensors and integrate gyro Z into heading. Call every tick."""
        now = time.ticks_us()
        dt = time.ticks_diff(now, self._last_us) / 1_000_000.0
        self._last_us = now
        if dt > 0.5 or dt <= 0:
            return

        raw = self.i2c.readfrom_mem(self.addr, _GYRO_XOUT, 6)
        gz_raw = struct.unpack('>h', raw[4:6])[0]
        # Negate to convert right-hand-rule physics convention (CCW positive
        # around Z-up) into compass / navigation convention (CW positive,
        # E=90°, S=180°, W=270°). Verified on a 360° hand-rotation loop test:
        # without this flip, heading runs 0→270→180→90→0 instead of
        # 0→90→180→270→0. All downstream consumers (heading integration,
        # dashboard display, agent prompt) expect compass-CW-positive.
        self.gyro_z = -(gz_raw - self._bias[2]) / _GYRO_SCALE

        # Read accel BEFORE bias logic — we need it for the stillness check.
        araw = self.i2c.readfrom_mem(self.addr, _ACCEL_XOUT, 6)
        ax = struct.unpack('>h', araw[0:2])[0] / _ACCEL_SCALE
        ay = struct.unpack('>h', araw[2:4])[0] / _ACCEL_SCALE
        az = struct.unpack('>h', araw[4:6])[0] / _ACCEL_SCALE
        self.accel = (round(ax, 3), round(ay, 3), round(az, 3))

        # Auto-bias correction — only fires when GENUINELY still. Two signals
        # required, both tight:
        #   1. Gyro magnitude < 0.3°/s (well below real motion).
        #   2. Accelerometer magnitude near 1g — confirms no translational
        #      force, no tilt change, no vibration. (1g² = 1.0; we accept
        #      ±5% which catches all stationary states without false-positive
        #      during coast-down or motor-vibration transients.)
        # Earlier version used gyro magnitude alone with a 1.5°/s threshold,
        # which falsely classified post-turn coast-down (~1°/s) as stillness
        # and absorbed real residual motion into bias. After multiple powered
        # turns the bias drifted enough that the IMU's reference frame
        # appeared to rotate. The accel check makes that impossible.
        accel_mag_sq = ax*ax + ay*ay + az*az
        gyro_quiet = abs(self.gyro_z) < 0.3
        accel_steady = 0.9 < accel_mag_sq < 1.1
        if gyro_quiet and accel_steady:
            self._still_count = min(self._still_count + 1, 1000)
        else:
            self._still_count = 0
        if self._still_count >= 50:   # ~250ms of detected stillness
            self._bias[2] += (gz_raw - self._bias[2]) * 0.005

        # Heading integration with a 0.75°/s deadzone. Empirically tuned: at
        # 1.0 the integration lost real slow-ramp motion (under-counted by
        # ~9° per 360°); at 0.5 residual bias leaked through (over-counted
        # by ~10°). 0.75 is the approximate zero-crossing point for
        # hand-rotation tests on this chassis.
        if abs(self.gyro_z) > 0.75:
            self._heading = (self._heading + self.gyro_z * dt) % 360.0

    @property
    def heading(self):
        return self._heading

    @heading.setter
    def heading(self, val):
        self._heading = val % 360.0

    def get_telemetry(self):
        """Returns [heading, gyro_z, ax, ay, az] for WebSocket transmission."""
        ax, ay, az = self.accel
        return [
            round(self._heading, 1),
            round(self.gyro_z, 2),
            round(ax, 2), round(ay, 2), round(az, 2),
        ]
