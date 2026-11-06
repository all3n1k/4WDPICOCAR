# Pico 4WD operating notes

These notes describe the current checked-in configuration. Older hardware
revisions used different I2C pins; use the firmware as the source of truth.

## Connections

| Component | Pico pins | Notes |
| --- | --- | --- |
| ESP8266 | GP4 TX / GP5 RX | UART1, 115200 baud |
| MPU6050 | GP0 SDA / GP1 SCL | SoftI2C, normally address 0x68 |
| VL53L0X | GP2 SDA / GP3 SCL | SoftI2C, address 0x29 |
| Sonar | GP6 trigger / GP7 echo | Swept by the radar servo |
| Radar servo | GP18 | Sweep limits in `examples/app_control.py` |
| Pan/tilt | GP20 / GP21 | Dashboard arrow keys |
| Chassis LEDs | GP19 | Motor/LED helpers in `libs/` |

Motor wiring and encoder assignments are defined in `libs/pico_4wd.py`
and `libs/pico_rdp.py`. The firmware stops the motors if no valid command
arrives within its 2000 ms command timeout. Do not treat this timeout as a
substitute for supervised testing.

## Configuration

1. Copy `examples/secrets.example.py` to the Pico filesystem as `secrets.py`.
   Fill in the SSID and password locally.
2. Configure the camera stream and WebSocket addresses in `main_os.py`.
3. Configure Ollama there if using `--auto`; manual mode does not need it.
4. Replace the installation-specific apartment prior and room-marker mapping
   with your own layout. Measure and survey markers before testing navigation.

Keep the chassis still during IMU startup calibration. Test motor direction
with the wheels lifted before driving on a surface.

## Control and mapping

Run `python3 main_os.py` for manual operation at http://localhost:8080.
The dashboard shows video, detections, heading, range sensors, and the map.
WASD drives, Shift increases manual power, and arrow keys move the camera.

`--carto` runs a mapping pass without the normal model decision loop.
`--map` runs the mapping routine. Both are development tools that depend on
working sensor calibration. Pose drift and incorrect marker measurements can
produce misplaced obstacles.

Boot clears the generated semantic and constellation maps unless
`--continue` is supplied. Manually surveyed wall markers and the static prior
are retained. Do not reuse stale sensor maps after moving the robot by hand.

## Protocol

The WebSocket bridge defaults to port 8765. Movement commands contain `K`
(direction) and `A` (power). For example:

```json
{"K": "forward", "A": 30}
{"K": "stop", "A": 0}
```

Current telemetry includes speed, mileage, radar samples, grayscale readings,
IMU data and ToF range. See `examples/app_control.py` for the emitted fields
and `dimos_lite/pico.py` for their desktop-side interpretation.

## Troubleshooting

- Missing IMU/ToF readings: check the pins above, scan the I2C bus, and inspect
  the firmware startup log. Both sensor paths use SoftI2C.
- Camera LED only switches on/off: some camera firmware variants do not expose
  brightness control. This requires a matching camera firmware implementation.
- Incorrect heading or map position: recalibrate with the robot still, verify
  marker scale/coordinates, and start a fresh mapping pass.
- No battery reading: this hardware configuration has no battery-voltage
  measurement. No voltage should be inferred from a placeholder value.
