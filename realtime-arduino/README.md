## Arduino ADS1263 Examples

Arduino sketches for the Waveshare High-Precision AD HAT (ADS1263), tested with **Arduino UNO** and **Arduino Mega 2560**.

### Sketches

- `arduino_self_check/arduino_self_check.ino`
  - Basic board health check (serial output, LED heartbeat, optional A0 read).
- `ads1263_reader/ads1263_reader.ino`
  - ADS1263 differential capture on `IN0-IN1`
  - DRDY falling-edge interrupt sampling
  - Continuous streaming output lines: `DATA,timestamp_us,raw,mV`
  - Periodic stream stats lines: `STATS,fps=...,count=...,dropped_edges=...`
- `bno055_reader/bno055_reader.ino`
  - BNO055 IMU over I2C (`SDA`/`SCL`), **no third-party IMU library** (raw register I²C)
  - **ACCGYRO** mode only; configurable accel G-range, gyro °/s range, and both sensors’ bandwidth (see `bno055_reader/BNO055_ACC_GYRO.md`)
  - Outputs CSV: `t_ms,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z` (units: mg, dps)
- `stream_logger.py`
  - Laptop-side logger that waits on serial stream and records CSV

### Wiring (UNO -> AD HAT)

- `D13 (SCK)` -> `SCLK`
- `D11 (MOSI)` -> `DIN`
- `D12 (MISO)` -> `DOUT`
- `D10` -> `CS`
- `D2` -> `DRDY`
- `D4` -> `RESET` / `REST`
- `5V` -> `AVDD` (analog supply)
- `3.3V` -> `VCC` (digital logic supply)
- `GND` -> `AVSS` (or `COM` if AVSS-COM is already tied)

### Wiring (Mega 2560 -> AD HAT)

- `D52 (SCK)` -> `SCLK`
- `D51 (MOSI)` -> `DIN`
- `D50 (MISO)` -> `DOUT`
- `D53` -> `CS`
- `D2` -> `DRDY`
- `D4` -> `RESET` / `REST`
- `5V` -> `AVDD` (analog supply)
- `3.3V` -> `VCC` (digital logic supply)
- `GND` -> `AVSS` (or `COM` if AVSS-COM is already tied)

### Important Electrical Note

The AD HAT is Raspberry-Pi-oriented (3.3V logic). UNO/Mega pins are 5V.
Use level shifting on outputs (`SCLK`, `DIN`, `CS`, `RESET`) for safe operation.

### Build and Upload (arduino-cli)

From project root:

```bash
cd /home/boshi/Documents/BowPressure
```

List boards/ports:

```bash
arduino-cli board list
```

Compile ADS1263 reader (pick the right FQBN for your board):

```bash
arduino-cli compile --fqbn <FQBN> realtime-arduino/ads1263_reader
```

Compile self-check (pick the right FQBN for your board):

```bash
arduino-cli compile --fqbn <FQBN> realtime-arduino/arduino_self_check
```

Upload ADS1263 reader:

```bash
arduino-cli upload -p /dev/ttyACM1 --fqbn <FQBN> realtime-arduino/ads1263_reader
```

Important: the `--fqbn` must match the exact board you are using.
For example:
- Arduino UNO R4 WiFi: `arduino:renesas_uno:unor4wifi`
- Arduino UNO (classic AVR): `arduino:avr:uno`
- Arduino Mega 2560 (AVR): `arduino:avr:mega`
- Do `arduino-cli board list` to see your fqbn.

Replace `/dev/ttyACM1` with the port shown by `arduino-cli board list` for your setup.

### Laptop Logger

Install dependency:

```bash
python -m pip install pyserial
```

Run logger (matches sketch baud rate `230400`):

```bash
python realtime-arduino/stream_logger.py --port /dev/ttyACM1 --baud 230400 --output ads1263_stream.csv
```

Optional timed run (example: 20 seconds):

```bash
python realtime-arduino/stream_logger.py --port /dev/ttyACM1 --baud 230400 --output ads1263_stream.csv --duration 20
```

### Expected ADS1263 Reader Output

- Register configuration verification lines (`REG_MODE*`, `REG_REFMUX`)
- Stream start message
- Repeating sample lines:
  - `DATA,timestamp_us,raw,mV`
- Periodic status lines:
  - `STATS,fps=...,count=...,dropped_edges=...`

### BNO055 (IMU) Wiring + Notes

- Connect IMU `SDA` -> Arduino `SDA` (UNO R4: labeled SDA, often D14)
- Connect IMU `SCL` -> Arduino `SCL` (UNO R4: labeled SCL, often D15)
- Connect IMU `3V3` (or `VIN` per breakout) and `GND` per your board datasheet
- Default I2C address is usually `0x28` (`BNO055_ADDRESS_A`); some boards use `0x29`

**UNO R4 WiFi** — compile/upload example (adapt port from `arduino-cli board list`):

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi realtime-arduino/bno055_reader
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi realtime-arduino/bno055_reader
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

No extra libraries are required for `bno055_reader` (uses `Wire` only).
