# Bow-v2 hardware and firmware reference

For the Tsinghua hardware, cable connections, programming, recording console,
host scripts, and external force calibration, use the [main README](../README.md).
This document covers the design archive, revision-specific details, and
implementation notes.

## Design archive

| Directory | Contents |
| --- | --- |
| [design/](design/) | EDA projects, complete design PDFs, and manufacturing exports |
| [firmware/](firmware/) | STM32 firmware, host tools, and tests |
| [specs/](specs/) | Component datasheets |
| [tools/](tools/) | EPRO2 inspection and design-PDF generation tools |

The electronics use an STM32C011F6P6 MCU, ADS122S14 ADC, ICM-45686 IMU,
and TPS7A20 regulator. Designs with LTC input use a TLV7011 comparator
for signal conditioning.

| Board revision | EDA project | Complete design and pin connections |
| --- | --- | --- |
| Board-v1 | [Project](design/v1.epro2) | [PDF](design/Bow-v1_Design_and_Pin_Connections.pdf) |
| Board-v2 | [Project](design/v2.epro2) | [PDF](design/Bow-v2_Design_and_Pin_Connections.pdf) |
| Board-v3 | [Project](design/v3.epro2) | [PDF](design/Bow-v3_Design_and_Pin_Connections.pdf) |
| Board-v3.5 | [Project](design/v3.5.epro2) | [PDF](design/Bow-v3.5_Design_and_Pin_Connections.pdf) |
| Board-v4 | [Project](design/v4.epro2) | [PDF](design/Bow-v4_Design_and_Pin_Connections.pdf) |

Each board revision has one complete PDF containing its component list and pin
connections. Existing PDF filenames are retained as archive identifiers;
select the design using the board-revision column above.

### Revision-specific notes

- **Board-v1** preserves known wiring errors: R7 pulls up ADC_DRDY, and IMU SDO
  is connected to ADC_CS. Its J1/J3 pin ordering also differs. The PDF documents
  the archived project; it is not a corrected design for manufacture.
- **Board-v3.5** adds R21, a 2.2 kOhm pull-down from J1.1 LTC to GND before C20.
  Bridge input nets are renamed H1/F1/F2 without changing connectivity.
- **Board-v4** has a differential communication interface. The current firmware
  directory does not contain a dedicated Board-v4 configuration.

### Manufacturing exports and PDF generation

Historical fabrication and placement exports are preserved with their original
export dates in [Board-v2 manufacturing exports](design/manufacturing/v2/)
and [Board-v3 manufacturing exports](design/manufacturing/v3/).
These are not regenerated Board-v3.5 fabrication files.

[tools/build_design_pdfs.py](tools/build_design_pdfs.py) generates the Board-v1,
Board-v2, and Board-v3.5 guides from the selected PCB and schematic in each EPRO2
project. It uses ReportLab and the macOS Arial Unicode font.
[tools/epro_model.py](tools/epro_model.py) provides the project-reading helpers.

## Firmware implementation notes

Firmware-v2 and Firmware-v3 share acquisition and serial-protocol code.
Hardware-specific definitions are selected through
[config/v2.h](firmware/config/v2.h) and
[config/v3.h](firmware/config/v3.h). The corresponding LTC-disabled builds use
the same hardware definitions. Firmware-v3 handles ADC DRDY and IMU FIFO
interrupts by deferring SPI transactions to the main loop.

For source changes, run the following from `firmware/` after installing the
build dependencies described in the main README:

```sh
make       # Build all four supported firmware targets.
make test  # Run the firmware project's automated checks.
```

Outputs are written to [firmware/build/](firmware/build/). Further implementation
and protocol details are in the [firmware reference](firmware/README.md).

## Diagnostic interpretation

| Observation | Interpretation and checks |
| --- | --- |
| CRC or sequence-error counters | Check whether they continue increasing. Inspect cable length, grounding, and power. Invalid-CRC packets are discarded. Counters at recording start may include earlier preview activity. |
| ADC fixed near ±103.125 mV | This is full scale with a 3.3 V reference and PGA gain 32. Check bridge excitation and midpoint connections; it is not a valid force measurement. |
| IMU FIFO full or fault value 4 | Check for excessive LTC interrupts, supply problems, or SPI wiring faults. After correcting the cause, reset and test again. |
| SWD connection failure | Try Connect under reset in STM32CubeProgrammer and check NRST, ground, supply, and SWD wiring. |
| Debugger firmware reported as too old | Use STM32CubeProgrammer's ST-LINK firmware-upgrade function, then reconnect. |

When programming a raw BIN instead of a HEX file, use flash start address
`0x08000000`.
