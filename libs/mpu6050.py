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

# ±250°/s → 131 LSB per °/s
_GYRO_SCALE = 131.0
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

        self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, b'\x00')
        time.sleep_ms(100)
        self.i2c.writeto_mem(self.addr, _GYRO_CONFIG, b'\x00')
        self.i2c.writeto_mem(self.addr, _ACCEL_CONFIG, b'\x00')
        # DLPF ~44Hz bandwidth — smooths noise without adding much lag
        self.i2c.writeto_mem(self.addr, _DLPF_CONFIG, b'\x03')

    def calibrate(self, samples=200):
        """Average gyro readings while stationary to find bias offset."""
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

    def update(self):
        """Read sensors and integrate gyro Z into heading. Call every tick."""
        now = time.ticks_us()
        dt = time.ticks_diff(now, self._last_us) / 1_000_000.0
        self._last_us = now
        if dt > 0.5 or dt <= 0:
            return

        raw = self.i2c.readfrom_mem(self.addr, _GYRO_XOUT, 6)
        gz_raw = struct.unpack('>h', raw[4:6])[0]
        self.gyro_z = (gz_raw - self._bias[2]) / _GYRO_SCALE

        # Dead-zone filter: ignore tiny readings (sensor noise when still)
        if abs(self.gyro_z) > 0.3:
            self._heading = (self._heading + self.gyro_z * dt) % 360.0

        araw = self.i2c.readfrom_mem(self.addr, _ACCEL_XOUT, 6)
        ax = struct.unpack('>h', araw[0:2])[0] / _ACCEL_SCALE
        ay = struct.unpack('>h', araw[2:4])[0] / _ACCEL_SCALE
        az = struct.unpack('>h', araw[4:6])[0] / _ACCEL_SCALE
        self.accel = (round(ax, 3), round(ay, 3), round(az, 3))

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
