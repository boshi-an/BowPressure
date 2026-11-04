#!/usr/bin/env python3
"""Plot external force, bowing fraction, and onboard bow reading in 3D."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


def load_samples(path: Path, bow_column: str):
    """Read recorded pairs without fitting, smoothing, or synthesizing samples."""
    required = ("force_N", "distance_fraction", bow_column)
    samples = []
    skipped = 0
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        missing = set(required) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"CSV is missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            try:
                force, fraction, bow = (float(row[key]) for key in required)
                if not all(math.isfinite(value) for value in (force, fraction, bow)):
                    raise ValueError("nonfinite value")
                if not 0 <= fraction <= 1:
                    raise ValueError("fraction outside 0..1")
            except (TypeError, ValueError):
                skipped += 1
                continue
            samples.append((force, fraction, bow))
    if not samples:
        raise ValueError("CSV has no valid calibration samples")
    if skipped:
        print(f"Skipped {skipped} invalid CSV rows.", file=sys.stderr)
    return samples


def recording_status(path: Path) -> str:
    """Metadata is optional, including for interrupted recordings."""
    metadata_path = path.with_suffix(".json")
    if not metadata_path.exists():
        return ""
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict):
            raise ValueError("expected a JSON object")
        status = metadata.get("status", "unknown")
        return f" | Run status: {status}"
    except (OSError, ValueError) as exc:
        print(f"Could not read optional metadata {metadata_path}: {exc}", file=sys.stderr)
        return ""


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="calibration CSV produced by calibrate.py")
    parser.add_argument("--output", type=Path,
                        help="PNG, SVG, or PDF path; default: <input-stem>-3d.png")
    parser.add_argument("--show", action="store_true", help="open a rotatable 3D plot window")
    parser.add_argument("--bow-units", choices=("mv", "raw24"), default="mv",
                        help="onboard full-bridge units: millivolts (default) or ADC counts")
    args = parser.parse_args(argv)
    output = args.output or args.csv.with_name(args.csv.stem + "-3d.png")
    if output.suffix.lower() not in (".png", ".svg", ".pdf"):
        parser.error("--output must end in .png, .svg, or .pdf")
    if output.resolve() == args.csv.resolve():
        parser.error("output must be different from input")
    bow_column = "fullbridge_mv" if args.bow_units == "mv" else "fullbridge_raw24"
    bow_label = "Bow full-bridge reading (mV)" if args.bow_units == "mv" else "Bow full-bridge reading (ADC counts)"
    try:
        samples = load_samples(args.csv, bow_column)
        status = recording_status(args.csv)
        import matplotlib
        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import Normalize
        from matplotlib.ticker import MaxNLocator

        force, fraction, bow = zip(*samples)
        with plt.rc_context({"font.size": 10, "axes.titlesize": 17}):
            fig = plt.figure(figsize=(11, 8), facecolor="white")
            ax = fig.add_subplot(111, projection="3d")
            points = ax.scatter(force, fraction, bow, c=fraction,
                                cmap="viridis", norm=Normalize(0, 1),
                                s=16, alpha=0.8, linewidths=0, depthshade=False)
            ax.set_xlabel("External force (N)", labelpad=12)
            ax.set_ylabel("Bowing fraction from frog", labelpad=12)
            ax.set_zlabel(bow_label, labelpad=12)
            ax.set_ylim(0, 1)
            ax.set_yticks([i / 10 for i in range(11)])
            ax.xaxis.set_major_locator(MaxNLocator(6))
            ax.zaxis.set_major_locator(MaxNLocator(6))
            ax.set_box_aspect((1.2, 1, 1))
            ax.view_init(elev=24, azim=-55)
            ax.set_title("Bow calibration", pad=24)
            colorbar = fig.colorbar(points, ax=ax, shrink=0.58, pad=0.12, aspect=25)
            colorbar.set_label("Bowing fraction: 0 = frog, 1 = tip")
            fig.text(0.5, 0.06,
                     f"{args.csv.name} | {len(samples):,} pairs | "
                     f"{len(set(fraction))} positions{status}", ha="center", fontsize=10)
            fig.text(0.5, 0.025, "Recorded pairs; onboard bridge readings are not calibrated force.",
                     ha="center", color="#555555", fontsize=9)
            fig.subplots_adjust(left=0.04, right=0.90, bottom=0.13, top=0.90)
            output.parent.mkdir(parents=True, exist_ok=True)
            try:
                fig.savefig(output, dpi=180, bbox_inches="tight", pad_inches=0.3)
                print(f"Saved {len(samples)} pairs across {len(set(fraction))} positions to {output}")
                if args.show:
                    plt.show()
            finally:
                plt.close(fig)
    except ImportError:
        print("Install plotting dependencies: python -m pip install -r requirements.txt", file=sys.stderr)
        return 1
    except (OSError, ValueError) as exc:
        print(f"Could not plot calibration: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
