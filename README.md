# Bow calibration

## Hardware preparation

The **Tsinghua hardware version** uses **V2 bows fitted with V3 boards and V3
firmware**: two cello bows (`v2-1`, `v2-2`) and two violin bows (`v2-3`, `v2-4`).
Use the V3 board documentation and firmware targets for these bows.

The supplied J1 data cable connects through the USB-TTL adapter for sensor
readings. The supplied J3 programming cable connects through the ST-LINK adapter
for firmware programming. See the [Tsinghua hardware guide](Bow-v2/README.md#tsinghua-hardware-version)
for details and calibration files.

The external force sensor used for Tsinghua hardware calibration is 艾德堡HP-50 from 京东.

## Setup

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
| Bow-v2/v3 | 1,500,000 baud, 8N1 |
| External force gauge | Modbus RTU, 9600 baud, 8N1, address 1 |

See [Bow-v2 hardware and firmware](Bow-v2/README.md) for board designs, wiring,
and firmware setup. Set the force gauge to live/track mode and zero it while unloaded.
Replace the example port names below with your device paths.

## Monitor external force

```sh
python monitor_force.py --port /dev/cu.FORCE_ADAPTER
```

Displays timestamped force in newtons at up to 10 Hz. Press Ctrl+C to stop.
Close the monitor before using the same port for calibration.

## Collect calibration data

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

## Plot recorded points

```sh
python plot_calibration.py recordings/bow1/bow-calibration.csv --show
```

Creates a 3D scatter plot with external force (N), bowing fraction, and
full-bridge reading (mV). Saves `<input-stem>-3d.png`; `--show` opens the plot.

## Fit a calibration model

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

## References and tests

- [Bow-v2 hardware and firmware](Bow-v2/README.md)
- [Force gauge protocol](manuals/Modbus_RTU规约_SH_通信协议.pdf)

```sh
python -m unittest discover -s tests -v
```
