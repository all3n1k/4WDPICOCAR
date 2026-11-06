# Pico 4WD robot

Raspberry Pi Pico firmware and a Python control stack for a four-wheel robot.
The desktop side combines an ESP32-CAM feed, YOLO object detection, ArUco
markers, sensor telemetry, and a browser dashboard. Optional Ollama-driven
autonomy runs on the desktop; motor and sensor control stay on the Pico.

This is a hardware prototype. Mapping accuracy, camera behavior, and autonomous
navigation still need tuning for each installation.

## Hardware

- Pico/RP2040 with the Pico-4WD motor, sonar, encoder, grayscale, and LED hardware.
- ESP8266 WebSocket bridge and a separate ESP32-CAM.
- MPU6050 on GP0/GP1 and VL53L0X on GP2/GP3, using SoftI2C.
- Pan/tilt servos on GP20/GP21.

The IMU calibrates at boot; hold the chassis still. There is no battery-voltage
sensor in this configuration.

## Setup

Clone the repository and create a Python environment on the desktop:

```sh
git clone https://github.com/all3n1k/4WDPICOCAR.git
cd 4WDPICOCAR
python3 -m venv .venv
source .venv/bin/activate
python -m pip install requests websockets numpy opencv-contrib-python ultralytics torch
```

Use MicroPython on the Pico. Copy the modules in `libs/` to the device, along
with `examples/app_control.py`. Copy `examples/secrets.example.py` to the
Pico as `secrets.py` and set your own Wi-Fi credentials. Keep that file out
of Git. The desktop and camera addresses in `main_os.py` must match your LAN.
The checked-in addresses and room layout are installation-specific examples.

Start with manual control:

```sh
python3 main_os.py
```

Open http://localhost:8080. WASD controls movement, Shift enables boost,
and the arrow keys control pan/tilt. Normal startup runs the dashboard, sensors,
and obstacle reflex without starting the model's decision loop.

## Operating modes

| Command | Purpose |
| --- | --- |
| `python3 main_os.py` | Manual controls, telemetry, vision, and reflex loop |
| `python3 main_os.py --auto` | Also start Ollama-driven decisions |
| `python3 main_os.py --carto` | Marker and obstacle mapping pass |
| `python3 main_os.py --map` | Autonomous mapping routine |
| `python3 main_os.py --continue` | Preserve the last runtime map on startup |

Normal startup clears `semantic_map.json` and `constellation_map.json`.
The surveyed `room_markers.json` and static `apartment_map.py` are preserved.
Configure your own floor plan and marker survey before relying on localization.

For `--auto`, run Ollama at the endpoint configured in `main_os.py` and
install the configured model. The current entry point selects `gemma4:26b`.
Model tool calls include movement and source-editing actions, so autonomy
should run in a supervised development environment.

## Documentation

- [Hardware, protocol, and operating notes](docs/USER_GUIDE.md)
- [Pico-4WD API reference](api_reference_pico_4wd.md)
- [Change history](CHANGELOG.md)

Known limitations include sensor noise, pose drift, and camera firmware variants
that only support binary LED control. The code is not a guarantee of collision
avoidance.
