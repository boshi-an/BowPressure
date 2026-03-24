## BowPressure

C++ utility for reading bow pressure data from a **CMCU-06A** strain-gauge transmitter (Modbus-RTU over USB-CDC) and logging it to CSV. Includes a Python script for plotting.

### Hardware

- **Transmitter**: CMCU-06A (Modbus-RTU, function codes 03/06/10)
- **Connection**: USB-CDC (`/dev/ttyACM0` on Linux, driver-free on Win10/11)
- **Serial config**: 38400 baud, 8N1 (baud rate is ignored over USB-CDC)

### Building

Requirements: `g++` with C++17 support, `make`.

```bash
make          # produces ./bowpressure
make clean    # remove binary
```

### Usage

```bash
./bowpressure [serial_port] [frame_rate_hz] [log_filename]
```

| Argument | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Path to the serial device |
| `frame_rate_hz` | `10.0` | Query rate in Hz (must be positive) |
| `log_filename` | `bowpressure_log.csv` | CSV output file (appended) |

Examples:

```bash
./bowpressure                                  # all defaults
./bowpressure /dev/ttyACM0 20                  # 20 Hz polling
./bowpressure /dev/ttyACM0 10 session1.csv     # custom log file
```

Press **Ctrl+C** to stop. The program prints a summary of total samples and errors on exit.

### What it does on startup

1. Opens the serial port and configures it.
2. Sends Modbus commands to the CMCU-06A to:
   - Disable write protection (register 23)
   - Set the internal ADC sampling rate (register 14)
   - Set the average filter depth (register 17)
   - Re-enable write protection
3. Enters the polling loop — reads registers 0–1 (32-bit signed final data) at the configured frame rate.

### CSV output format

```text
timestamp,raw_value
2026-03-06 17:40:50.674,1234
2026-03-06 17:40:50.775,-56
```

- `timestamp` — local time, `YYYY-MM-DD HH:MM:SS.mmm`
- `raw_value` — signed 32-bit integer from the Modbus response

The live console output also shows current FPS, sample count, and error count.

### Plotting

Requires Python 3 with `pandas` and `matplotlib`.

```bash
python plot.py                      # plots bowpressure_log.csv
python plot.py session1.csv         # plots a specific file
```

Displays elapsed time (seconds) on the x-axis and the raw sensor value on the y-axis.
