# BowPressure — Bow Sensor Recording and Force Calibration

## On-bow electronics: usage and connections

The **Tsinghua hardware version** uses **Bow-v2** bows fitted with **Board-v3**
and **Firmware-v3**: two cello bows (**Bow-v2-1**, **Bow-v2-2**) and two violin
bows (**Bow-v2-3**, **Bow-v2-4**). The supplied J1 data cable and USB-TTL adapter
are used for recording. The supplied J3 programming cable and **Waveshare mini
ST-LINK/V2** are used for programming. Use the Board-v3 documentation and
Firmware-v3 build targets for these bows.

**Both cables are provided:** the J1 data cable and the J3 programming cable.
At the **USB-TTL and ST-LINK adapter ends**, insert each plug with its
**small arrow facing up**.

See the [board design reference](Bow-v2/README.md#design-archive)
and the supplied [calibration files](recordings/calibrations.zip).

### Power and data connections

The following J1 wiring applies to Board-v2, Board-v3, and Board-v3.5:

| Board connector | Signal | Connection |
| --- | --- | --- |
| J1.1 | LTC | 25 fps LTC audio source; share ground with the board |
| J1.2 | GND | USB-TTL GND |
| J1.3 | DATA | USB-TTL RX, using 3.3 V logic |
| J1.4 | 5V | USB 5 V power supply |

Leave USB-TTL TX disconnected. Power the board through J1 during both recording
and programming. Board-v4 uses differential communication
and does not use this USB-TTL data connection.

### Programming and reset

For the Tsinghua hardware, use the supplied Waveshare mini ST-LINK/V2 and J3 cable:

| Board connector | Waveshare mini ST-LINK/V2 signal |
| --- | --- |
| J3.1 — GND | GND |
| J3.3 — NRST | RST / NRST |
| J3.4 — SWCLK | SWCLK |
| J3.5 — SWDIO | SWDIO |

**Connect both cables during programming:** connect J1 to the USB-TTL adapter
for board power, and J3 to the ST-LINK adapter for programming. Connect both
adapters to the computer and keep both cables connected throughout programming.

Open STM32CubeProgrammer, select ST-LINK and SWD, connect, and program the
matching HEX file. Verify the download, then reset the MCU. Reset is also possible
by briefly connecting NRST to GND and releasing it, or by cycling power.

| Hardware | Firmware target | HEX file, relative to `Bow-v2/firmware/` |
| --- | --- | --- |
| Board-v2 | Firmware-v2 | `build/v2.hex` |
| Board-v2, LTC disabled | Firmware-v2_no_ltc | `build/v2_no_ltc.hex` |
| Board-v3 or Board-v3.5 | Firmware-v3 | `build/v3.hex` |
| Board-v3 or Board-v3.5, LTC disabled | Firmware-v3_no_ltc | `build/v3_no_ltc.hex` |

For Tsinghua hardware, use Firmware-v3 when receiving LTC, or Firmware-v3_no_ltc
when LTC is not needed. A floating LTC input can generate unwanted interrupts;
the LTC-disabled build avoids processing these interrupts.

The nominal IMU rate is 1600 Hz, with each ADC bridge sampled at 1000 samples/s.
Serial communication uses **1,500,000 baud, 8N1**. Existing HEX files can be flashed
directly. To rebuild, install Make and `arm-none-eabi-gcc`, then run `make v3`
or `make v3_no_ltc` from `Bow-v2/firmware/`.

## Recording console and host scripts

### Environment and console startup

Start from the **BowPressure root directory**:

```sh
# Create the environment once; skip this line if stm32 already exists.
conda env create -f Bow-v2/firmware/environment.yml
conda activate stm32
cd Bow-v2/firmware
python host/recorder_gui.py
```

The service opens a local webpage and automatically starts previewing the
preferred USB serial device. Use the URL printed in the terminal if the browser
does not open. `--no-browser` suppresses automatic browser opening;
`--port 8080` selects the HTTP server port, not the serial device.

### Preview and recording

1. Select the serial device and baud rate while previewing. The default is
   1,500,000 baud; this selection does not reconfigure the MCU.
2. Choose a parent output directory and a new recording folder name.
3. Start recording. Earlier preview data is not saved.
4. Pause to stop saving while continuing to preview; resume to save new data.
   Paused intervals are not filled in later.
5. End recording to finish the files and generate `merged.csv`. Preview resumes
   automatically afterward.

Four plot rows display ACC and GYRO with three axes each, followed by half-bridge
H and full-bridge F in millivolts. Gray background marks recorded intervals.
Plots use reduced display data, while CSV files retain the received samples
without host-side smoothing. Each plot uses a vertical span of 120% of its
minimum-to-maximum range. The latest-readings panel updates once per second.

Serial settings are locked during recording and pausing. Only one program may
open the serial device at a time. Closing the webpage does not stop the service;
press **Ctrl+C in its terminal** to stop it and finalize an active recording.

### Command-line recording

Run these commands from **BowPressure/Bow-v2/firmware**:

```sh
python host/read.py --list
python host/read.py --port /dev/cu.usbserial-BG03EL9E --output out/run1 --diagnostics
```

Replace the example serial path with your device. The output directory must not
already exist. Press Ctrl+C to finish, or add `--duration 120` for a timed run.
Omit `--output` for monitoring without saving. The default baud rate is 1,500,000;
use `--baud` only when it matches the firmware configuration.
Stop the console service before running this reader on the same serial device.

### Saved data and time mapping

| File | Contents |
| --- | --- |
| `adc.csv` | Half-bridge and full-bridge raw counts, millivolts, status, and MCU timestamps |
| `imu.csv` | Acceleration, angular velocity, sensor timestamps, and mapped MCU timestamps |
| `ltc.csv` | Timecodes, lock state, counters, and timing anchors |
| `mcu.csv` | MCU status, FIFO diagnostics, errors, and startup messages |
| `merged.csv` | Combined records with LTC time mapping, generated when recording finishes |

The ADC and IMU source files contain no LTC columns. IMU sensor timestamps are
mapped to MCU time; ADC sample-time estimates account for the configured
conversion/filter delay. Readout time is separate from estimated sample time.

The merged file retains separate rows for measurements and status records,
without resampling or filling one sensor's values into another sensor's rows.
LTC time is represented in integer microseconds. The shared mapper interpolates
or extrapolates through unlocked intervals where usable anchors exist, and
records the method in `sync`. MCU runs without usable anchors retain blank LTC
times; anchors are not borrowed across MCU resets. Microsecond representation
does not imply ±1 µs physical synchronization accuracy.

To regenerate the merged file after recording:

```sh
python host/export_ltc.py out/run1
```

This uses the same mapping code as automatic export and leaves the four source
CSV files unchanged.

### Plotting and LTC test audio

```sh
python host/plot.py out/run1 --output out/run1/plot.png --show
python host/plot.py out/run1/merged.csv --output out/run1/merged_plot.png
python host/generate_ltc.py test_signals/my_ltc.wav --seconds 60 --amplitude 0.25
```

The plotting script accepts a recording directory or merged CSV and draws ACC,
GYRO, H, and F against MCU time. Use `--epoch` to select a MCU run when a recording
contains resets. The output extension selects PNG, PDF, or SVG; `--show` opens an
interactive plot.

The LTC generator creates a 25 fps, 48 kHz mono WAV file; it does not play it.
Connect the audio source to J1.1 and share GND. The amplitude argument is a digital
WAV amplitude, not a guaranteed voltage at the audio output.

### Host file reference

Paths below are relative to `Bow-v2/firmware/`.

| File | Purpose |
| --- | --- |
| `host/recorder_gui.py` | Local recording console service |
| `host/recorder.html` | Console frontend, loaded by the service rather than opened directly |
| `host/read.py` | Serial monitoring and recording; `--list` lists devices and `--output` selects the recording directory |
| `host/export_ltc.py` | Shared LTC mapping and standalone merged-file export |
| `host/plot.py` | Offline ACC, GYRO, H, and F plots |
| `host/generate_ltc.py` | 25 fps LTC test-audio generation |
| `host/run_output.py` | Internal CSV writer and automatic export integration |
| `host/imu_time.py` | Internal IMU-to-MCU clock mapping |
| `host/sample_time.py` | Internal ADC sample-time estimation |

Internal modules do not need to be run directly. Use `--help` on command-line
scripts for options. See the [Bow-v2 hardware and firmware guide](Bow-v2/README.md)
for additional wiring and troubleshooting details.

## Bow calibration with an external force sensor

The Tsinghua calibration setup uses an **Aidebao HP-50** external force gauge,
purchased from JD.com. Connect it and the bow to separate serial adapters.

Run the following commands from the **BowPressure root directory**. If currently
in `Bow-v2/firmware/`, run `cd ../..` first. Stop the recording console with Ctrl+C
before calibration so that it releases the bow serial device.
Calibration uses its own paired-data CSV format and plotting scripts, separate
from the four-file sensor recording workflow above.

### Setup

Use Python 3.10 or newer:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python calibrate.py --list-ports
```

Connect each sensor to its own serial port:

| Device | Serial settings |
| --- | --- |
| Bow-v2 with Board-v3 and Firmware-v3 | 1,500,000 baud, 8N1 |
| External force gauge | Modbus RTU, 9600 baud, 8N1, address 1 |

See [Bow-v2 hardware and firmware](Bow-v2/README.md) for board designs, wiring,
and firmware setup. Set the force gauge to live/track mode and zero it while unloaded.
Replace the example port names below with your device paths.

### Monitor external force

```sh
python monitor_force.py --port /dev/cu.FORCE_ADAPTER
```

Displays timestamped force in newtons at up to 10 Hz. Press Ctrl+C to stop.
Close the monitor before using the same port for calibration.

### Collect calibration data

For a bow with 0.61 m usable length:

```sh
python calibrate.py \
  --bow-port /dev/cu.BOW_ADAPTER \
  --force-port /dev/cu.FORCE_ADAPTER \
  --bow-length-m 0.61 \
  --output recordings/bow1/bow-calibration.csv
```

1. Keep bow tension, mounting, and force direction consistent.
2. Move the contact point to the prompted position and press Enter.
3. After the one-second settling period, slowly increase and decrease the force
   while keeping the contact position fixed.
4. Repeat for all ten positions. Check the printed force range at each position.

Each position records 100 pairs at up to 10 Hz. Adjust these with
`--samples-per-position`, `--poll-hz`, and `--settle-s`.
For a 0.61 m bow, positions run from 30.5 to 579.5 mm from the frog.

The CSV contains position, full-bridge counts and millivolts, external force in
newtons, timestamps, and raw packets. The companion JSON records settings and
run status. Samples are paired using host receipt times. Type `q` at a prompt
or press Ctrl+C to stop; collected data stays saved. Use a fresh output filename
for each run.

### Plot recorded points

```sh
python plot_calibration.py recordings/bow1/bow-calibration.csv --show
```

Creates a 3D scatter plot with external force (N), bowing fraction, and
full-bridge reading (mV). Saves `<input-stem>-3d.png`; `--show` opens the plot.

### Fit a calibration model

```sh
python fit_calibration.py recordings/bow1/bow-calibration.csv --show
python fit_calibration.py recordings/bow1/bow-calibration.csv --model bilinear --show
```

Here `x` is external force (N), `y` is bowing fraction, and `z` is full-bridge
reading (mV).

| Model | Function | Output stem |
| --- | --- | --- |
| `simple` (default) | `z = a*x*y + c` | `<input-stem>-fit` |
| `bilinear` | `z = a*x*y + b*x + c*y + d` | `<input-stem>-fit-bilinear` |

Both models use least squares and exclude rows where external force equals zero.
The JSON saves coefficients and fit metrics: RMSE, MAE, maximum absolute error,
and R². The PNG overlays the fitted surface and the measured points used for
fitting.

With `--show`, left-drag to rotate, middle-drag to pan, and right-drag to zoom.
Keep toolbar Pan/Zoom modes off while rotating. Close the window to finish.

Use `--bow-units raw24` for ADC counts, `--output` for the fit JSON path, and
`--plot-output` for the fitted plot path. Run any script with `--help` for options.

### References and tests

- [Bow-v2 hardware and firmware](Bow-v2/README.md)
- [Force gauge protocol](manuals/Modbus_RTU规约_SH_通信协议.pdf)

```sh
python -m unittest discover -s tests -v
```
