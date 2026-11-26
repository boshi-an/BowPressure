# Firmware implementation reference

See the [main README](../../README.md) for setup, flashing, recording, and host
script usage, and the [Bow-v2 design reference](../README.md) for hardware files.

## Source layout and build controls

| Path | Responsibility |
| --- | --- |
| `config/v2.h`, `config/v3.h` | Select Board-v2 or Board-v3 hardware definitions |
| `src/main.c`, `src/startup.c` | MCU initialization and interrupt vectors |
| `src/fast_*.inc` | Acquisition, scheduling, LTC, and UART transport |
| `src/imu_fifo_dma.inc` | Board-v3 SPI DMA for IMU FIFO reads |
| `src/fast_protocol.h` | Packet framing and CRC |
| `host/serial_rx.py` | Dedicated receiver and byte-bounded 1 MiB queue |
| `host/raw_capture.py` | Capture and replay serial bytes |
| `host/imu_time.py`, `host/sample_time.py` | Sensor timestamp estimation |
| `host/export_ltc.py` | Shared LTC mapping and merged-file export |
| `tests/` | Host and native firmware-logic checks |
| `vendor/` | STM32/CMSIS headers and licenses |

Firmware-v2 and Firmware-v3 share acquisition code; their LTC-disabled variants
are compiled with `DISABLE_LTC_IRQ`. This masks PA0 edge interrupts while leaving
PA0 as an input without a pull resistor. Sensor acquisition remains enabled.

Run `make v3 v3_no_ltc` to build the standard 3 Mbaud images in `build/`.
The Makefile defaults to `SPI_BR=3` (3 MHz) for polled IMU transfers.
`make v3 SPI_BR=4` sets those transfers to 1.5 MHz. The Board-v3 FIFO
DMA path uses its own 6 MHz SPI setting.

`make test` runs the automated checks. `make clean` removes only `build/`,
leaving recordings untouched. Builds do not flash the board.

## Acquisition and scheduling

| Component | Current configuration |
| --- | --- |
| ICM-45686 | 3200 Hz low-noise mode; 20-bit High Resolution FIFO format with fixed ±32 g / ±4000 °/s scaling; AAF and 800 Hz UI low-pass filter |
| ADS122S14 | Mode 3, OSR 256, PGA 32, AVDD reference; alternating bridges in 500 µs slots |
| Board-v2 IMU FIFO | 2 KiB, 20 bytes per frame, up to 102 complete frames |
| Board-v3 IMU FIFO | 8 KiB, 20 bytes per frame, up to 409 complete frames and an eight-frame interrupt watermark |

Firmware-v2 polls IMU availability and ADC DRDY. Firmware-v3 uses IMU INT1 on
PA6 and ADC DRDY falling edges on PA3, with level checks and fallback polling.
The IMU fallback interval is 5 ms.

TIM3 schedules ADC conversions. ADC DRDY records readiness and timing, IMU
interrupts set pending flags, and LTC interrupts enqueue edge timestamps.
The main loop schedules transfers and decodes LTC. Firmware-v3 gives LTC a
bounded turn before IMU work within the ADC timing guard.

Board-v3 SPI DMA reads up to eight complete IMU frames per burst at 6 MHz
when the ADC deadline allows it. SPI ownership returns before ADC work.
The main loop packs the buffered samples into UART packets. The watermark
can increase delivery latency and cause samples to arrive in bursts. IMU
sample timestamps and the 3.2 kHz sampling rate are preserved. Firmware-v2
reads up to four frames per pass and yields to the ADC between frames.

## Transport and diagnostics

Packets are 32 bytes, with little-endian multibyte fields:

```text
A5 5A | protocol version (2) | type | sequence u32 | read_us u32
      | payload (18 bytes) | CRC16 (2 bytes)
```

CRC16-CCITT-FALSE covers bytes 0–29. Packet types are IMU (1), ADC (2), status
(3), LTC status (4), IMU diagnostics (5), and LTC edge anchor (6). Protocol
version 2 is independent of the board and firmware target names.
Both boards use UART DMA. Firmware-v3 uses the STM32 CRC peripheral after
a startup self-test, with a software fallback.

Fault flags are a bitmask:

| Bit value | Meaning |
| --- | --- |
| `0x01` | ADC initialization, communication, or timeout failure |
| `0x02` | IMU status/count read failure |
| `0x04` | FIFO full or count at least 102 on Board-v2 or 409 on Board-v3. Reset to resume IMU reading. |
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

With `--diagnostics` (enabled by the recorder GUI), the receive reports in
`mcu.csv` also include bytes handed to the decoder, time since the last read,
the longest gap between nonempty reads, peak driver pending bytes, and peak
time spent waiting in the application queue. Read gaps also reflect USB
batching and host scheduling. Use MCU timestamps and packet sequences to
verify that a gap represents missing data.

The same rows include per-report decoded and saved packet counts, decoder
discarded and pending bytes, and elapsed wall time in decoding, processing,
CSV writes, and flushes. Processing time includes CSV writes. The GUI shows
these measurements as `# RX` and `# PIPE`. The existing `crc_errors` counts
rejected header or CRC candidates, and `decoder_header_errors` counts the
header subset.

`# GAP` rows mark sequence discontinuities, ADC/IMU intervals of at least
10 ms, and intervals of at least 20 ms with no decoded packet following active
sensor streaming. Each gap includes MCU endpoints and a host receive-queue
snapshot taken when the parser observes it. `# STREAM` rows mark serial
silence, resumption, or a receive error or MCU timestamp discontinuity.

## Raw serial capture and replay

The recorder and replay tool can read type-7 `DIAG` packets from archived
captures. These records include boot reset flags, maximum loop gap, completed
UART DMA bytes, peak TX-ring occupancy, and LTC interrupt counts.

### Capture raw serial and CSV together

```sh
python host/recorder_gui.py --raw-capture
python host/read.py --port /dev/cu.usbserial-BG03EL9E --baud 3000000 \
  --duration 60 --diagnostics --raw-capture --output out/v3_capture
```

Use one recorder per port. The optional raw capture keeps a bounded prefix
in RAM (32 MiB or 1,250,000 serial reads, whichever comes first) before the
application receive queue and saves `serial.bin`,
`rx_chunks.csv`, and `rx_capture.json` after serial reception stops.
It includes bytes later rejected by the decoder or dropped by the
application receive queue. A GUI capture promoted from Preview starts its
raw copy when recording begins. The manifest records its offset from
port open and any truncation. CSV recording continues if the raw limit is reached.

### Capture with no live decoding or CSV writes

```sh
python host/raw_capture.py capture --port /dev/cu.usbserial-BG03EL9E \
  --baud 3000000 --duration 60 --output out/v3_raw
python host/raw_capture.py replay out/v3_raw
```

Replay also accepts the directory from a combined capture. Compare its
CRC and sequence totals with the saved records over the same time interval.
Use the boot markers and MCU counters to check whether the bow reset or
stopped producing packets when the UART stream went quiet.
