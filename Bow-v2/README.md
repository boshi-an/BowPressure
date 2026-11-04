# Bow hardware and firmware

Electronics for measuring bow strain and motion, with half-bridge and full-bridge
inputs, a six-axis IMU, and an LTC timecode input. The firmware streams readings
to a computer over UART.

## Hardware design

| Folder | Contents |
| --- | --- |
| [design/](design/) | EDA projects, connection guides, bills of materials, Gerbers, and placement files |
| [firmware/](firmware/) | STM32 firmware, serial readers, plotting tools, and tests |
| [specs/](specs/) | Component datasheets |

The board uses an STM32C011F6P6 microcontroller, ADS122S14 ADC, ICM-45686 IMU,
and TPS7A20 regulator. A TLV7011 comparator conditions the LTC input.

Design files are organized by board revision:

| Revision | EDA project | Design reference |
| --- | --- | --- |
| V1 | [v1.epro2](design/v1.epro2) | [Design and pin connections](design/Bow-v1_Design_and_Pin_Connections.pdf) |
| V2 | [v2.epro2](design/v2.epro2) | [Design and pin connections](design/Bow-v2_Design_and_Pin_Connections.pdf) |
| V3 | [v3.epro2](design/v3.epro2) | [Design and pin connections](design/Bow-v3_Design_and_Pin_Connections.pdf) |
| V3.5 | [v3.5.epro2](design/v3.5.epro2) | [Design and pin connections](design/Bow-v3.5_Design_and_Pin_Connections.pdf) |
| V4 | [v4.epro2](design/v4.epro2) | [Design and pin connections](design/Bow-v4_Design_and_Pin_Connections.pdf) |

Each revision has one complete PDF. V1, V2 and V3.5 guides were regenerated from
those projects on 2026-11-04; V3/V4 retain their existing guides. Firmware notes
in the older V3/V4 PDFs may predate the current code; use `firmware/README.md`
for current acquisition and shared CSV behavior.

V3.5 adds R21 (2.2 kOhm) from J1.1 LTC to GND, before C20; bridge input nets are
renamed H1/F1/F2 with unchanged connectivity. It uses the V3 firmware pin mapping.
V1 preserves known errors: R7 pulls up ADC_DRDY, and IMU SDO is connected to
ADC_CS. Its J1/J3 pin ordering also differs; the V1 PDF documents the actual
project and must not be treated as a corrected design for manufacture.

Historical manufacturing exports are preserved in
[manufacturing/v2](design/manufacturing/v2/) and
[manufacturing/v3](design/manufacturing/v3/), with original export dates.
These exports are not regenerated V3.5 fabrication files. Superseded split V2
BOM/connection PDFs and the outdated review text have been removed.
The reproducible PDF generator is `tools/build_design_pdfs.py` (ReportLab,
using the macOS Arial Unicode font); it reads the selected PCB and schematic
from each EPRO2 and regenerates the V1/V2/V3.5 guides.

## Firmware

The firmware targets V2 and V3 hardware. Both acquire each bridge at a target
rate of 1,000 samples/s and IMU data at 1,600 Hz. Serial output uses
**1,500,000 baud, 8N1**.

| Build target | Board | LTC |
| --- | --- | --- |
| `v2` | V2 | Enabled |
| `v2_no_ltc` | V2 | Disabled |
| `v3` | V3 | Enabled |
| `v3_no_ltc` | V3 | Disabled |

Choose the target matching your board and LTC connection. V3 uses the IMU FIFO
interrupt and a status LED. Hardware settings are in `firmware/config/`;
acquisition and protocol code are in `firmware/src/`.

### Build and flash

Install the Arm GNU toolchain (`arm-none-eabi-gcc`) and Make. From this `Bow-v2`
directory, prepare the Python environment and build the desired target:

```sh
conda env create -f firmware/environment.yml
conda activate stm32
cd firmware
make v3_no_ltc
make test
```

`make` builds all four targets. Outputs are saved in `firmware/build/`.
Use STM32CubeProgrammer and an ST-LINK connected over SWD to flash the matching
`.hex` file, verify it, and reset the board.

Follow the [firmware guide](firmware/README.md) for J1 power/data wiring,
J3 programming connections, and diagnostics.

### Read and plot sensor data

Run from `Bow-v2/firmware/` with the Python environment active:

```sh
python host/read.py --list
python host/read.py --port /dev/cu.BOW_ADAPTER --diagnostics
```

Record 60 seconds and plot the result:

```sh
python host/read.py --port /dev/cu.BOW_ADAPTER --duration 60 --csv out/record01.csv
python host/plot.py out/record01.csv --show
```

Replace the port with your device path and use a fresh CSV filename for each run.
The recording contains bridge readings, IMU data, timecode, and device status.
Press Ctrl+C to stop recording.

For paired external-force calibration and model fitting, see the
[project README](../README.md).
