#!/usr/bin/env python3
"""GUI logger for ADS1263 stream with start/stop and output path selection."""

from __future__ import annotations

# pylint: disable=missing-class-docstring,missing-function-docstring
# pylint: disable=too-many-instance-attributes,too-many-branches
# pylint: disable=too-many-statements,too-few-public-methods
# pylint: disable=consider-using-with

import csv
import json
import os
import queue
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from acceleration_compensation.inference import RealtimeCompensator, load_model
from stream_logger import (
    ProcessedSample,
    StreamDataProcessor,
    StreamPlotter,
)
from streamer import SerialLineGenerator, parse_data_line, parse_data_parts


@dataclass
class LoggerStats:
    sample_count: int = 0
    bad_lines: int = 0
    status: str = "Idle"


class LiveCsvRecorder:
    _HEADER_ADS = [
        "host_iso_time",
        "host_epoch_s",
        "arduino_timestamp_us",
        "raw",
        "mV_raw",
        "mV_ema",
        "ltc_timecode",
    ]
    _HEADER_ADS_IMU = [
        "host_iso_time",
        "host_epoch_s",
        "arduino_timestamp_us",
        "raw",
        "mV_raw",
        "mV_ema",
        "acc_x_mg",
        "acc_y_mg",
        "acc_z_mg",
        "gyr_x_dps",
        "gyr_y_dps",
        "gyr_z_dps",
        "aang_x_raw_dps2",
        "aang_y_raw_dps2",
        "aang_z_raw_dps2",
        "aang_x_ema_dps2",
        "aang_y_ema_dps2",
        "aang_z_ema_dps2",
        "ltc_timecode",
    ]

    def __init__(self, output_path: str, stream_format: str) -> None:
        self._file: Any = open(output_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        if stream_format == "ads":
            self._writer.writerow(self._HEADER_ADS)
        else:
            self._writer.writerow(self._HEADER_ADS_IMU)

    def write_sample(self, sample: ProcessedSample) -> None:
        self._writer.writerow(sample.csv_row)

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()


class GraphicsLoggerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ADS1263 Graphics Logger")

        self.port_var = tk.StringVar(value="/dev/ttyACM1")
        self.baud_var = tk.StringVar(value="230400")
        self.output_var = tk.StringVar(
            value=str(Path.cwd() / "ads1263_stream.csv")
        )
        self.model_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Idle")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.recording_var = tk.StringVar(value="Not recording")
        self.count_var = tk.StringVar(value="0")
        self.bad_var = tk.StringVar(value="0")

        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._connected = False
        self._recording = False
        self._plotter: Optional[StreamPlotter] = None
        self._first_arduino_us: Optional[int] = None
        self._record_output: Optional[str] = None
        self._record_count_reset_pending = False
        self._known_ports: list[str] = []
        self._port_display_to_address: dict[str, str] = {}
        self._close_requested = False
        self._queue_after_id: Optional[str] = None
        self._arduino_cli_missing_warned = False

        self._build_ui()
        self._refresh_ports()
        self._bring_window_to_front()
        self._pump_queue()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Serial Port").grid(row=0, column=0, sticky="w", pady=4)
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, state="normal")
        self.port_combo.grid(
            row=0, column=1, sticky="ew", pady=4
        )
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(
            row=0, column=2, padx=(8, 0), pady=4
        )

        ttk.Label(frame, text="Baud").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(frame, textvariable=self.baud_var).grid(
            row=1, column=1, sticky="ew", pady=4
        )

        ttk.Label(frame, text="Output CSV").grid(row=2, column=0, sticky="w", pady=4)
        ttk.Entry(frame, textvariable=self.output_var).grid(
            row=2, column=1, sticky="ew", pady=4
        )
        ttk.Button(frame, text="Browse...", command=self._select_output_path).grid(
            row=2, column=2, padx=(8, 0), pady=4
        )
        ttk.Label(frame, text="Compensation Model").grid(
            row=3, column=0, sticky="w", pady=4
        )
        ttk.Entry(frame, textvariable=self.model_var).grid(
            row=3, column=1, sticky="ew", pady=4
        )
        ttk.Button(frame, text="Browse...", command=self._select_model_path).grid(
            row=3, column=2, padx=(8, 0), pady=4
        )

        self.connect_button = ttk.Button(
            frame, text="Connect", command=self._toggle_connection
        )
        self.connect_button.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(10, 4))
        self.record_button = ttk.Button(
            frame, text="Start recording", command=self._toggle_recording, state="disabled"
        )
        self.record_button.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(4, 10))

        stats = ttk.LabelFrame(frame, text="Session")
        stats.grid(row=6, column=0, columnspan=3, sticky="ew")
        stats.columnconfigure(1, weight=1)

        ttk.Label(stats, text="Status").grid(row=0, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(stats, textvariable=self.status_var).grid(
            row=0, column=1, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, text="Connection").grid(
            row=1, column=0, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, textvariable=self.connection_var).grid(
            row=1, column=1, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, text="Recording").grid(
            row=2, column=0, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, textvariable=self.recording_var).grid(
            row=2, column=1, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, text="Logged Samples").grid(
            row=3, column=0, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, textvariable=self.count_var).grid(
            row=3, column=1, sticky="w", padx=8, pady=4
        )
        ttk.Label(stats, text="Bad Lines").grid(
            row=4, column=0, sticky="w", padx=8, pady=(4, 8)
        )
        ttk.Label(stats, textvariable=self.bad_var).grid(
            row=4, column=1, sticky="w", padx=8, pady=(4, 8)
        )

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _select_output_path(self) -> None:
        selected = filedialog.asksaveasfilename(
            title="Select CSV output path",
            initialfile=os.path.basename(self.output_var.get()),
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if selected:
            self.output_var.set(selected)

    def _toggle_connection(self) -> None:
        if self._connected:
            self._disconnect()
            return
        self._connect()

    def _select_model_path(self) -> None:
        selected = filedialog.askopenfilename(
            title="Select fitted model JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if selected:
            self.model_var.set(selected)

    def _list_arduino_ports(self) -> list[tuple[str, str]]:
        try:
            proc = subprocess.run(
                ["arduino-cli", "board", "list", "--format", "json"],
                check=True,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            parsed = json.loads(proc.stdout)
        except FileNotFoundError:
            if not self._arduino_cli_missing_warned:
                print(
                    "Warning: 'arduino-cli' command not found; auto port detection disabled.",
                    file=sys.stderr,
                )
                self._arduino_cli_missing_warned = True
            return []
        except (subprocess.SubprocessError, json.JSONDecodeError):
            return []

        ports: list[tuple[str, str]] = []
        for item in parsed.get("detected_ports", []):
            port_info = item.get("port", {})
            address = str(port_info.get("address", "")).strip()
            matching_boards = item.get("matching_boards", [])
            board_name = "Unknown"
            if matching_boards:
                board_name = str(matching_boards[0].get("name", "Unknown")).strip() or "Unknown"
            if address:
                ports.append((address, board_name))
        return ports

    def _refresh_ports(self) -> None:
        ports = self._list_arduino_ports()
        current = self.port_var.get().strip()
        values: list[str] = []
        self._port_display_to_address = {}
        preferred_display: Optional[str] = None
        for address, board_name in ports:
            display = f"[{board_name}] {address}"
            values.append(display)
            self._port_display_to_address[display] = address
            if preferred_display is None and board_name != "Unknown":
                preferred_display = display
        if current and current not in values and current not in self._port_display_to_address:
            values.insert(0, current)
        self._known_ports = values
        self.port_combo["values"] = values
        if (
            preferred_display is not None
            and (not current or current == "/dev/ttyACM1" or current not in values)
        ):
            self.port_var.set(preferred_display)

    def _connect(self) -> None:
        port_choice = self.port_var.get().strip()
        port = self._port_display_to_address.get(port_choice, port_choice)
        baud_raw = self.baud_var.get().strip()

        if not port:
            messagebox.showerror("Missing serial port", "Please provide a serial port.")
            return
        try:
            baud = int(baud_raw)
        except ValueError:
            messagebox.showerror("Invalid baud", f"Baud must be an integer, got: {baud_raw}")
            return

        self.status_var.set("Connecting...")
        self.connection_var.set("Connecting...")
        self.connect_button.configure(state="disabled")
        self._stop_event.clear()
        self._plotter = StreamPlotter(plot_window_s=5.0)
        self._plotter.set_recording_indicator(False, reset=True)
        self._first_arduino_us = None
        self._record_output = None
        self._recording = False
        self._record_count_reset_pending = False
        self.recording_var.set("Not recording")
        self.count_var.set("0")
        self.bad_var.set("0")

        self._worker = threading.Thread(
            target=self._stream_worker,
            args=(port, baud),
            daemon=False,
        )
        self._worker.start()

    def _disconnect(self) -> None:
        if not self._connected:
            return
        self.status_var.set("Disconnecting...")
        self._connected = False
        self.connection_var.set("Disconnected")
        if self._plotter is not None:
            self._plotter.close()
            self._plotter = None
        if self._recording:
            self._recording = False
            self.recording_var.set("Not recording")
            self.record_button.configure(text="Start recording")
        self._stop_event.set()
        self.status_var.set("Disconnected")
        self.connect_button.configure(text="Connect", state="normal")
        self.record_button.configure(text="Start recording", state="disabled")

    def _toggle_recording(self) -> None:
        if not self._connected:
            messagebox.showwarning("Not connected", "Connect to device before recording.")
            return
        if self._recording:
            self._recording = False
            self._record_output = None
            self.recording_var.set("Not recording")
            self.record_button.configure(text="Start recording")
            self.status_var.set("Connected (not recording)")
            if self._plotter is not None:
                self._plotter.set_recording_indicator(False)
            return

        output = self.output_var.get().strip()
        if not output:
            messagebox.showerror("Missing output path", "Please choose an output CSV path.")
            return
        output_path = Path(output)
        if output_path.exists():
            overwrite = messagebox.askyesno(
                "File already exists",
                f"The log file already exists:\n\n{output_path}\n\nOverwrite it?",
                icon="warning",
            )
            if not overwrite:
                return
        if output_path.parent and not output_path.parent.exists():
            output_path.parent.mkdir(parents=True, exist_ok=True)

        self.count_var.set("0")
        self._record_output = str(output_path)
        self._recording = True
        self._record_count_reset_pending = True
        self.recording_var.set("Recording")
        self.record_button.configure(text="Stop recording")
        self.status_var.set("Connected (recording)")
        if self._plotter is not None:
            self._plotter.set_recording_indicator(True, reset=True)

    def _stream_worker(self, port: str, baud: int) -> None:
        stats = LoggerStats(status="Running")
        self._queue.put(("stats", stats))
        stream_format: Optional[str] = None
        processor = StreamDataProcessor()
        compensator: Optional[RealtimeCompensator] = None
        model_path_raw = self.model_var.get().strip()
        if model_path_raw:
            try:
                loaded_model = load_model(Path(model_path_raw))
                compensator = RealtimeCompensator(
                    loaded=loaded_model,
                    mv_ema_alpha=0.20,
                    aang_ema_alpha=0.20,
                )
                self._queue.put(
                    ("stats", LoggerStats(status=f"Running (compensation: {loaded_model.model_type})"))
                )
            except Exception as exc:  # pylint: disable=broad-except
                self._queue.put(("error", f"Failed to load compensation model: {exc}"))
                self._queue.put(("connected", False))
                self._queue.put(("stopped", None))
                return
        last_flush_at = time.time()
        saver: Optional[LiveCsvRecorder] = None
        connected_notified = False

        try:
            with SerialLineGenerator(port, baud) as reader:
                for line in reader.lines(stop_event=self._stop_event):
                    if line.startswith("STATS,"):
                        continue
                    parsed_line = parse_data_line(line)
                    if parsed_line is None:
                        continue
                    line_stream_format, parts = parsed_line
                    decoded = parse_data_parts(parts, line_stream_format)
                    if decoded is None:
                        stats.bad_lines += 1
                        self._queue.put(("stats", LoggerStats(**vars(stats))))
                        continue

                    if stream_format is None:
                        stream_format = line_stream_format
                        self._queue.put(("plot_format", stream_format))

                    if line_stream_format != stream_format:
                        stats.bad_lines += 1
                        self._queue.put(("stats", LoggerStats(**vars(stats))))
                        continue

                    sample = processor.process(decoded)
                    if sample is None:
                        stats.bad_lines += 1
                        self._queue.put(("stats", LoggerStats(**vars(stats))))
                        continue

                    compensated_mv: Optional[float] = None
                    if compensator is not None:
                        observed = compensator.observe(decoded)
                        if observed is not None:
                            center, pred_mv = observed
                            if center.arduino_us == sample.arduino_us:
                                compensated_mv = sample.mv - pred_mv

                    self._queue.put(("sample", (stream_format, sample, compensated_mv)))
                    if not connected_notified:
                        self._queue.put(("connected", True))
                        connected_notified = True
                    if self._recording and stream_format is not None:
                        if self._record_count_reset_pending:
                            stats.sample_count = 0
                            self._record_count_reset_pending = False
                            self._queue.put(("stats", LoggerStats(**vars(stats))))
                        if saver is None:
                            assert self._record_output is not None
                            saver = LiveCsvRecorder(self._record_output, stream_format)
                        saver.write_sample(sample)
                        stats.sample_count += 1
                        if stats.sample_count % 50 == 0:
                            self._queue.put(("stats", LoggerStats(**vars(stats))))
                    elif saver is not None:
                        saver.close()
                        saver = None

                    now = time.time()
                    if saver is not None and now - last_flush_at >= 1.0:
                        saver.flush()
                        last_flush_at = now
                if saver is not None:
                    saver.flush()
                    saver.close()

        except Exception as exc:  # pylint: disable=broad-except
            self._queue.put(("error", str(exc)))
        finally:
            self._queue.put(("connected", False))
            if not self._stop_event.is_set():
                stats.status = "Stopped"
            else:
                stats.status = "Stopped by user"
            self._queue.put(("stats", LoggerStats(**vars(stats))))
            self._queue.put(("stopped", None))

    def _pump_queue(self) -> None:
        if self._close_requested:
            return
        while True:
            try:
                event, payload = self._queue.get_nowait()
            except queue.Empty:
                break

            if event == "stats":
                stats = payload
                assert isinstance(stats, LoggerStats)
                self.count_var.set(str(stats.sample_count))
                self.bad_var.set(str(stats.bad_lines))
                self.status_var.set(stats.status)
            elif event == "error":
                assert isinstance(payload, str)
                self.status_var.set("Error")
                print(f"[error] {payload}")
                messagebox.showerror("Logging failed", payload)
            elif event == "connected":
                assert isinstance(payload, bool)
                if payload and self._stop_event.is_set():
                    # Ignore stale "connected" events that race with disconnect.
                    continue
                self._connected = payload
                self.connection_var.set("Connected" if payload else "Disconnected")
                if payload:
                    self.status_var.set("Connected (not recording)")
                    self.connect_button.configure(text="Disconnect", state="normal")
                    self.record_button.configure(state="normal")
                else:
                    if self._plotter is not None:
                        self._plotter.close()
                        self._plotter = None
                    self.status_var.set("Disconnected")
                    self.recording_var.set("Not recording")
                    self.connect_button.configure(text="Connect", state="normal")
                    self.record_button.configure(text="Start recording", state="disabled")
            elif event == "plot_format":
                assert isinstance(payload, str)
                if self._plotter is not None and self._plotter.enabled:
                    self._plotter.ensure_layout(payload)
            elif event == "sample":
                stream_format, sample, compensated_mv = payload
                assert isinstance(stream_format, str)
                assert isinstance(sample, ProcessedSample)
                if self._first_arduino_us is None:
                    self._first_arduino_us = sample.arduino_us
                if self._plotter is not None and self._plotter.enabled:
                    self._plotter.on_sample(
                        stream_format=stream_format,
                        first_arduino_us=self._first_arduino_us,
                        sample=sample,
                        compensated_mv=compensated_mv,
                    )
            elif event == "stopped":
                self._recording = False
                self._record_output = None

        try:
            self._queue_after_id = self.root.after(100, self._pump_queue)
        except tk.TclError:
            # Window already closed; ignore pending reschedule.
            self._queue_after_id = None

    def _on_close(self) -> None:
        if self._close_requested:
            return
        self._close_requested = True
        self.status_var.set("Closing...")
        self.connect_button.configure(state="disabled")
        self.record_button.configure(state="disabled")
        self._stop_event.set()
        self._finalize_close()

    def _finalize_close(self) -> None:
        worker = self._worker
        if worker is not None and worker.is_alive():
            self.root.after(50, self._finalize_close)
            return
        if self._queue_after_id is not None:
            try:
                self.root.after_cancel(self._queue_after_id)
            except tk.TclError:
                pass
            self._queue_after_id = None
        self.root.destroy()

    def _bring_window_to_front(self) -> None:
        # macOS/Tk sometimes starts app windows behind other apps.
        self.root.update_idletasks()
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()
        self.root.after(250, self.root.lift)


def main() -> int:
    root = tk.Tk()
    GraphicsLoggerApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
