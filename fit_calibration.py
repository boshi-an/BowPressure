#!/usr/bin/env python3
"""Fit simple or bilinear bow calibration models, excluding zero-force readings."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


MODELS = {"simple": "z = a*x*y + c", "bilinear": "z = a*x*y + b*x + c*y + d"}


def predict(result, x, y):
    """Evaluate either model for scalar values or NumPy arrays."""
    if result.get("model_type", "simple") == "bilinear":
        return result["a"] * x * y + result["b"] * x + result["c"] * y + result["d"]
    return result["a"] * x * y + result["c"]


def equation(result, precision=6):
    text = f"z = {result['a']:.{precision}g} xy"
    if result.get("model_type", "simple") == "bilinear":
        return (text + f" {result['b']:+.{precision}g} x"
                f" {result['c']:+.{precision}g} y {result['d']:+.{precision}g}")
    return text + f" {result['c']:+.{precision}g}"


def load_samples(path: Path, bow_column: str):
    samples = []
    counts = dict(rows_read=0, zero_force_excluded=0, invalid_rows_excluded=0)
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"force_N", "distance_fraction", bow_column}
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"CSV is missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            counts["rows_read"] += 1
            try:
                x = float(row["force_N"])
                if not math.isfinite(x):
                    raise ValueError("nonfinite force")
                # Zero is the gauge's missing-force sentinel, not a baseline.
                if x == 0:
                    counts["zero_force_excluded"] += 1
                    continue
                y, z = float(row["distance_fraction"]), float(row[bow_column])
                if not all(math.isfinite(v) for v in (y, z)) or not 0 <= y <= 1:
                    raise ValueError("invalid fraction or bridge reading")
                samples.append((x, y, z))
            except (TypeError, ValueError):
                counts["invalid_rows_excluded"] += 1
    return samples, counts


def fit_model(samples, model="simple"):
    """Equal-weight ordinary least squares, rejecting unidentifiable models.

    The simple model uses centered covariance; the bilinear model uses NumPy's
    least-squares solver with columns [x*y, x, y, 1] and requires rank four.
    """
    if model not in MODELS:
        raise ValueError(f"Unknown model: {model}")
    minimum = 4 if model == "bilinear" else 2
    if len(samples) < minimum:
        raise ValueError(f"At least {minimum} valid nonzero-force samples are required")
    product = [x * y for x, y, _ in samples]
    z = [row[2] for row in samples]
    if not all(math.isfinite(v) for v in (*product, *z)):
        raise ValueError("Nonfinite model input")
    mean_z = math.fsum(z) / len(z)
    if model == "bilinear":
        import numpy as np
        design = np.array([[x * y, x, y, 1] for x, y, _ in samples], dtype=float)
        if not np.isfinite(design).all():
            raise ValueError("Nonfinite model input")
        try:
            coefficients, _, rank, _ = np.linalg.lstsq(design, z, rcond=None)
        except np.linalg.LinAlgError as exc:
            raise ValueError(f"Least-squares solver failed: {exc}") from exc
        if rank < 4:
            raise ValueError("Cannot identify a, b, c, d: [x*y, x, y, 1] must have rank 4; "
                             "collect independently varied force and bowing fraction")
        result = dict(zip(("a", "b", "c", "d"), map(float, coefficients)))
    else:
        mean_product = math.fsum(product) / len(product)
        centered = [v - mean_product for v in product]
        variance_sum = math.fsum(v * v for v in centered)
        if max(product) == min(product) or variance_sum == 0:
            raise ValueError("Cannot identify a and c: external force * fraction must vary")
        a = math.fsum(t * (value - mean_z) for t, value in zip(centered, z)) / variance_sum
        result = {"a": a, "c": mean_z - a * mean_product}
    evaluation = {**result, "model_type": model}
    residuals = [value - predict(evaluation, x, y) for x, y, value in samples]
    squared_error = math.fsum(r * r for r in residuals)
    total_variation = math.fsum((value - mean_z) ** 2 for value in z)
    result.update({
        "samples_used": len(samples),
        "rmse": math.sqrt(squared_error / len(samples)),
        "mae": math.fsum(abs(r) for r in residuals) / len(samples),
        "max_abs_error": max(abs(r) for r in residuals),
        "r_squared": 1 - squared_error / total_variation if total_variation > 0 else None,
    })
    if not all(value is None or math.isfinite(value) for value in result.values()):
        raise ValueError("Nonfinite fit result; check input magnitudes")
    return {**result, "model_type": model, "model": MODELS[model]}


def plotting_pyplot(show):
    """Use a desktop canvas for --show, even after an IDE selected inline/Agg."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if show:
        desktop_backends = {
            "macosx", "tkagg", "tkcairo", "qtagg", "qtcairo", "qt5agg", "qt5cairo",
            "gtk3agg", "gtk3cairo", "gtk4agg", "gtk4cairo", "wx", "wxagg", "wxcairo",
        }
        if matplotlib.get_backend().lower() not in desktop_backends:
            candidates = (["MacOSX"] if sys.platform == "darwin" else []) + ["TkAgg", "QtAgg"]
            failures = []
            for backend in candidates:
                try:
                    plt.switch_backend(backend)
                    break
                except (ImportError, RuntimeError) as exc:
                    failures.append(f"{backend}: {exc}")
            else:
                raise RuntimeError(
                    "--show needs a desktop GUI backend. Run from a local terminal with "
                    "Tkinter or PyQt6 installed, or omit --show to save the PNG. "
                    + "; ".join(failures)
                )
    return plt


def plot_fit(samples, result, output: Path, *, show=False):
    """Overlay the fitted model and the exact observations retained for fitting."""
    plt = plotting_pyplot(show)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    import numpy as np

    x, y, z = np.asarray(samples, dtype=float).T
    units = result["units"]["z"]
    model_label = "Fitted surface: " + result.get("model", MODELS["simple"]).replace("*", "")
    with plt.rc_context({"font.size": 10}):
        fig = plt.figure(figsize=(11, 9), facecolor="white")
        try:
            ax = fig.add_subplot(111, projection="3d")
            ax.mouse_init(rotate_btn=1, pan_btn=2, zoom_btn=3)
            if np.ptp(x) > 0 and np.ptp(y) > 0:
                grid_x, grid_y = np.meshgrid(np.linspace(x.min(), x.max(), 45),
                                             np.linspace(y.min(), y.max(), 45))
                grid_z = predict(result, grid_x, grid_y)
                ax.plot_surface(grid_x, grid_y, grid_z, color="#32a4b4", alpha=0.35,
                                edgecolor="#36717b", linewidth=0.2, shade=False)
                model_handle = Patch(facecolor="#32a4b4", alpha=0.45, label=model_label)
            else:
                # One force or position gives a line, not a sampled surface domain.
                grid_x = np.linspace(x.min(), x.max(), 100)
                grid_y = np.linspace(y.min(), y.max(), 100)
                model_label = "Fitted curve (one input is constant)"
                ax.plot(grid_x, grid_y, predict(result, grid_x, grid_y),
                        color="#168a9a", linewidth=2.5)
                model_handle = Line2D([], [], color="#168a9a", linewidth=2.5, label=model_label)
            ax.scatter(x, y, z, color="#a33232", s=13, alpha=0.8,
                       depthshade=False, linewidths=0)
            measured_handle = Line2D([], [], marker="o", linestyle="none", color="#a33232",
                                     markersize=5, label=f"Measured points (n={len(samples):,})")
            ax.legend(handles=[measured_handle, model_handle], loc="upper left", frameon=False)
            ax.set_xlabel("External force (N)", labelpad=12)
            ax.set_ylabel("Bowing fraction from frog", labelpad=12)
            ax.set_zlabel(f"Bow full-bridge reading ({units})", labelpad=12)
            ax.set_box_aspect((1.2, 1, 1))
            ax.view_init(elev=24, azim=-55)
            fig.suptitle("Fitted calibration surface vs. measured points", fontsize=17, y=0.96)
            fig.text(0.5, 0.91, f"{equation(result)}  [{units}]",
                     ha="center", fontsize=13)
            r_squared = "undefined" if result["r_squared"] is None else f"{result['r_squared']:.4f}"
            fig.text(0.5, 0.065, f"RMSE = {result['rmse']:.4g} {units}  |  R² = {r_squared}  |  "
                     f"Excluded zero-force rows: {result['zero_force_excluded']}", ha="center")
            fig.text(0.5, 0.03, "Model shown within observed input ranges; zero-force points are omitted.",
                     ha="center", fontsize=9, color="#555555")
            fig.subplots_adjust(left=0.04, right=0.85, bottom=0.15, top=0.87)
            output.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(output, dpi=180, bbox_inches="tight", pad_inches=0.35)
            print(f"Saved fitted surface and measured points to {output}")
            if show:
                fig.canvas.manager.set_window_title("Bow calibration - drag to rotate")
                print("Interactive plot: left-drag to rotate, middle-drag to pan, "
                      "right-drag to zoom. Turn off toolbar Pan/Zoom modes to rotate. "
                      "Close the window to finish.")
                plt.show(block=True)
        finally:
            plt.close(fig)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="calibration recording CSV")
    parser.add_argument("--model", choices=tuple(MODELS), default="simple",
                        help="simple: z=axy+c (default); bilinear: z=axy+bx+cy+d")
    parser.add_argument("--bow-units", choices=("mv", "raw24"), default="mv",
                        help="fit millivolts (default) or raw ADC counts")
    parser.add_argument("--output", type=Path,
                        help="JSON output; default: <input-stem>-fit[-bilinear].json")
    parser.add_argument("--plot-output", type=Path,
                        help="PNG, SVG, or PDF plot; default: JSON output path with .png extension")
    parser.add_argument("--show", action="store_true", help="also open a rotatable 3D plot window")
    args = parser.parse_args(argv)
    suffix = "-fit-bilinear.json" if args.model == "bilinear" else "-fit.json"
    output = args.output or args.csv.with_name(args.csv.stem + suffix)
    if output.suffix.lower() != ".json":
        parser.error("--output must end in .json")
    if output.resolve() in (args.csv.resolve(), args.csv.with_suffix(".json").resolve()):
        parser.error("output must not overwrite the recording or its metadata")
    plot_output = args.plot_output or output.with_suffix(".png")
    if plot_output.suffix.lower() not in (".png", ".svg", ".pdf"):
        parser.error("--plot-output must end in .png, .svg, or .pdf")
    if plot_output.resolve() == args.csv.resolve():
        parser.error("plot must not overwrite the input")
    column = "fullbridge_mv" if args.bow_units == "mv" else "fullbridge_raw24"
    units = "mV" if args.bow_units == "mv" else "ADC counts"
    try:
        samples, counts = load_samples(args.csv, column)
        result = fit_model(samples, args.model)
        coefficient_units = {"a": f"{units}/N", "c": units}
        if args.model == "bilinear":
            coefficient_units.update(b=f"{units}/N", d=units)
        result.update({
            "schema_version": 1,
            "source_csv": str(args.csv.resolve()),
            "columns": {"x": "force_N", "y": "distance_fraction", "z": column},
            "units": {"x": "N", "y": "fraction", "z": units,
                      **coefficient_units},
            "method": "ordinary least squares, equal weight per retained row",
            "filter": "force_N != 0; finite values; 0 <= distance_fraction <= 1",
            "error_units": units,
            "metrics_scope": "in-sample fit errors on retained rows",
            **counts,
            "force_range_N": [min(s[0] for s in samples), max(s[0] for s in samples)],
            "bowing_fractions": sorted({s[1] for s in samples}),
        })
        payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload, encoding="utf-8")
    except ImportError:
        print("Install fitting dependencies: python -m pip install -r requirements.txt", file=sys.stderr)
        return 1
    except (OSError, ValueError, OverflowError) as exc:
        print(f"Could not fit calibration: {exc}", file=sys.stderr)
        return 1
    print(f"Used {result['samples_used']}/{counts['rows_read']} rows; "
          f"excluded {counts['zero_force_excluded']} zero-force and "
          f"{counts['invalid_rows_excluded']} invalid rows.")
    print(f"{equation(result, precision=10)} [{units}]")
    r_squared = "undefined (constant z)" if result["r_squared"] is None else f"{result['r_squared']:.6g}"
    print(f"RMSE: {result['rmse']:.6g} {units}; MAE: {result['mae']:.6g} {units}; "
          f"max error: {result['max_abs_error']:.6g} {units}; R²: {r_squared}")
    print(f"Saved coefficients and fit metrics to {output}")
    try:
        plot_fit(samples, result, plot_output, show=args.show)
    except ImportError:
        print("Fit JSON was saved, but plotting dependencies are missing. "
              "Run: python -m pip install -r requirements.txt", file=sys.stderr)
        return 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Fit JSON was saved, but plotting failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
