# Firmware implementation reference

See the [main README](../../README.md) for setup, flashing, recording, and host
script usage, and the [Bow-v2 design reference](../README.md) for hardware files.

## Source layout and build controls

| Path | Responsibility |
| --- | --- |
| `config/v2.h`, `config/v3.h` | Select Board-v2 or Board-v3 hardware definitions |
| `src/main.c`, `src/startup.c` | MCU initialization and interrupt vectors |
| `src/fast_*.inc` | Acquisition, scheduling, LTC, diagnostics, and UART transport |
| `src/fast_protocol.h` | Packet framing and CRC |
| `host/serial_rx.py` | Dedicated receiver and byte-bounded 1 MiB queue |
| `host/imu_time.py`, `host/sample_time.py` | Sensor timestamp estimation |
| `host/export_ltc.py` | Shared LTC mapping and merged-file export |
| `tests/` | Host and native firmware-logic checks |
| `vendor/` | STM32/CMSIS headers and licenses |

Firmware-v2 and Firmware-v3 share acquisition code; their LTC-disabled variants
are compiled with `DISABLE_LTC_IRQ`. This masks PA0 edge interrupts while leaving
PA0 as an input without a pull resistor. Sensor acquisition remains enabled.

The Makefile defaults to `SPI_BR=3` (3 MHz). For a 1.5 MHz SPI build, use
`make v3 SPI_BR=4`. `make test` runs the automated checks; `make clean` removes
only `build/`, leaving recordings untouched. Builds do not flash the board.

## Acquisition and scheduling

| Component | Current configuration |
| --- | --- |
| ICM-45686 | 1600 Hz low-noise mode; 20-bit High Resolution FIFO format with fixed ±32 g / ±4000 °/s scaling; AAF and 400 Hz UI low-pass filter |
| ADS122S14 | Mode 3, OSR 256, PGA 32, AVDD reference; alternating bridges in 500 µs slots |
| IMU FIFO | 2048 bytes, 20 bytes per frame: 102 complete frames; two-frame interrupt watermark |

Firmware-v2 polls IMU availability and ADC DRDY. Firmware-v3 uses IMU INT1 on
PA6 and ADC DRDY falling edges on PA3, with level checks and fallback polling.
The IMU fallback interval is 5 ms.

TIM3 registers ADC conversion tasks. ADC DRDY records readiness and timing;
IMU interrupts set pending flags; LTC interrupts enqueue edge timestamps.
SPI transfers and LTC decoding run in the main loop. Interrupts are briefly
masked for shared-state handoff, not throughout SPI transfers.

IMU service reads at most four frames per pass, yields to ADC between frames,
and defers reads within 200 µs of the next ADC deadline. The two-frame watermark
adds approximately one sample interval of waiting to the oldest frame without
changing the sensor's sampling rate.

## Transport and diagnostics

Packets are 32 bytes, with little-endian multibyte fields:

```text
A5 5A | protocol version (2) | type | sequence u32 | read_us u32
      | payload (18 bytes) | CRC16 (2 bytes)
```

CRC16-CCITT-FALSE covers bytes 0–29. Packet types are IMU (1), ADC (2), status
(3), LTC status (4), IMU diagnostics (5), and LTC edge anchor (6). Protocol
version 2 is independent of the board and firmware target names.

Fault flags are a bitmask:

| Bit value | Meaning |
| --- | --- |
| `0x01` | ADC initialization, communication, or timeout failure |
| `0x02` | IMU status/count read failure |
| `0x04` | FIFO full or count at least 102; IMU reading stops pending reset |
| `0x08` | FIFO frame transfer or header failure |
| `0x10` | Invalid IMU sample marker |
| `0x20` | Empty FIFO with no sample for over 100 ms |
| `0x40` | UART TX error reported in status |
| `0x80` | Firmware-v3 has observed no IMU interrupt after the startup check; fallback polling remains available |

Fault flags latch until reset. Firmware-v3 drives the active-low PA4 LED with a
short healthy heartbeat or a 5 Hz fault indication. Missing LTC alone does not
trigger the fault indication.

## Timestamp implementation

**IMU:** `host/imu_time.py` unwraps the 16-bit sensor clock and selects low-delay
anchors in one-second windows. Up to 32 anchors are retained; clock-rate fitting
uses anchor pairs at least four seconds apart and starts after eight windows.
Offset corrections are limited to avoid timestamp jumps. Sequence gaps, invalid
sensor intervals, or MCU resets start a new mapping segment.

**ADC:** `host/sample_time.py` estimates effective sample time from observed
DRDY readiness minus the digital-filter group delay. For the current Mode 3,
OSR 256 configuration:

```text
sinc4(OSR 32) + sinc1(OSR 8)
delay = [4 × (32 − 1) + 32 × (8 − 1)] / (2 × 1.024 MHz)
      = 169.921875 µs
```

Firmware-v3 prefers the DRDY interrupt timestamp; Firmware-v2 and fallback paths
use main-loop readiness observation. Update this delay model when changing the
ADC configuration. The packet's `mcu_read_us` remains a readout timestamp.

**LTC:** the edge ISR feeds a 128-entry ring with 127 usable entries. Records
older than 50 ms are discarded to avoid ambiguous timer wrapping. Decoding
supports forward 25 fps LTC. Type 6 carries the final bit's end edge, which marks
the next frame's start; the host therefore adds 40,000 µs to the decoded timecode
for the anchor.

`host/export_ltc.py` owns the shared mapping implementation. It separates MCU
reset epochs and timecode discontinuities. Across gaps, positive LTC/MCU time
increments with a ratio between 0.95 and 1.05 and a MCU interval below 12 hours
are treated as potentially continuous. This heuristic can miss small timecode
jumps; a single anchor permits only nominal 1:1 extrapolation.

These are estimated sample times, not hardware synchronization guarantees.
Residual FIFO latency, IMU filter delay, analog filtering, comparator delay,
and interrupt jitter require measurement before claiming absolute accuracy.

## Host receive queue

`SerialReceiver` is the sole serial reader. It reads up to 4096 bytes per call
and queues immutable chunks under a condition lock, with a 1,048,576-byte payload
limit. Python object overhead and the in-flight chunk are additional memory.
The consumer retains decoding, CSV writing, clock mapping, and GUI callbacks.
On overflow the newest chunk is dropped without blocking reception; a generation
marker ensures the decoder clears a partial packet before crossing a loss.
Device errors propagate after already queued chunks have been processed.
The receive thread is joined before closing the port and finalizing output files.

Diagnostics use `rx_queue_bytes`, `rx_queue_capacity`, `rx_queue_peak_bytes`,
`rx_received_bytes`, `rx_overflow_events`, and `rx_dropped_bytes` in LOG records.
Periodic snapshots and overflow notifications share the CLI/GUI report path.
Overflow counts are per discarded read chunk, not per sensor packet. Final
occupancy reports unprocessed bytes intentionally left when recording stops.
