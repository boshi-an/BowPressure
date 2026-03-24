## Arduino ADS1263 Examples

Arduino sketches for the Waveshare High-Precision AD HAT (ADS1263), tested with **Arduino Mega 2560**.

### Sketches

- `arduino_self_check/arduino_self_check.ino`
  - Basic board health check (serial output, LED heartbeat, optional A0 read).
- `ads1263_reader/ads1263_reader.ino`
  - ADS1263 differential capture on `IN0-IN1`
  - DRDY falling-edge interrupt sampling
  - Buffered capture and end-of-run stats (`std(mV)`, `mean/std dt_us`, realized FPS)

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

The AD HAT is Raspberry-Pi-oriented (3.3V logic). Mega pins are 5V.
Use level shifting on Mega outputs (`SCLK`, `DIN`, `CS`, `RESET`) for safe operation.

### Build and Upload (arduino-cli)

From project root:

```bash
cd /home/boshi/Documents/BowPressure
```

List boards/ports:

```bash
arduino-cli board list
```

Compile self-check:

```bash
arduino-cli compile --fqbn arduino:avr:mega realtime-arduino/arduino_self_check
```

Upload self-check (replace port if needed):

```bash
arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:avr:mega realtime-arduino/arduino_self_check
```

Compile ADS1263 reader:

```bash
arduino-cli compile --fqbn arduino:avr:mega realtime-arduino/ads1263_reader
```

Upload ADS1263 reader:

```bash
arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:avr:mega realtime-arduino/ads1263_reader
```

Serial monitor (115200):

```bash
arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
```

### Expected ADS1263 Reader Output

- Register configuration verification lines (`REG_MODE*`, `REG_REFMUX`)
- Capture start message
- End-of-capture summary and CSV-like sample dump:
  - `timestamp_us,raw,mV`
  - final stats including realized FPS
