# PicoBot

Educational robotics platform for the Embedded Systems (ES101) course at Obuda University.

## Quick Start

### Deploy to Pico

```bash
# Upload the picobot library
mpremote connect /dev/ttyACM0 cp -r lib/picobot/ :lib/picobot/
mpremote connect /dev/ttyACM0 cp lib/bmi160.py :lib/bmi160.py

# Upload a lab exercise
mpremote connect /dev/ttyACM0 cp labs/lab3/main.py :main.py
```

### Host Tools

```bash
cd host
pip install -r requirements.txt
python viewer.py
```

## Structure

- `lib/` — MicroPython libraries (deploy to Pico)
- `labs/` — Progressive lab exercises
- `examples/` — Standalone example programs
- `tests/` — Unit tests
- `calibration/` — Motor and sensor calibration
- `host/` — PC-side tools and visualization

## Course Documentation

Full course documentation: https://www.aut.uni-obuda.hu/es/
