import binascii
import csv
from dataclasses import replace
import json
from pathlib import Path
import struct
import tempfile
import time
import unittest
from unittest.mock import patch

from bowcal.bow import BowDecoder, BowSample, BowStream
from bowcal.calibration import POSITIONS, Recording, parse_args, run_sequence
from bowcal.force import ForceGauge, ProtocolError, UNITS, read_request, response_payload, with_crc


def bow_packet(raw=-123456, channel=1, seq=1, us=1000, status=0xFF60, gain=32, kind=2):
    payload = struct.pack("<iHBBII", raw, status, channel, gain, (us - 400) & 0xFFFFFFFF, 400)
    data = struct.pack("<2sBBII", b"\xa5\x5a", 2, kind, seq, us) + payload.ljust(18, b"\0")
    return data + struct.pack("<H", binascii.crc_hqx(data, 0xFFFF))


def force_frame(value=1.25, unit=0, address=1):
    data = struct.pack(">f", value) + bytes(18) + struct.pack(">H", unit)
    return with_crc(bytes([address, 3, len(data)]) + data)


class ForcePort:
    baudrate = 9600
    timeout = 0.02

    def __init__(self, frame=None):
        self.frame = force_frame() if frame is None else frame
        self.pending = bytearray()
        self.writes = []

    def reset_input_buffer(self):
        self.pending.clear()

    def write(self, data):
        self.writes.append(data)
        self.pending.extend(self.frame)
        return len(data)

    def read(self, size):
        # Deliberately fragment every response.
        if not self.pending:
            time.sleep(min(self.timeout, 0.002))
        result = bytes(self.pending[:min(size, 3)])
        del self.pending[:len(result)]
        return result


class BowTests(unittest.TestCase):
    def test_signed_fullbridge_and_units(self):
        sample, = BowDecoder().feed(bow_packet(raw=-4194304))
        self.assertEqual(sample.raw24, -4194304)
        self.assertEqual(sample.gain, 32)
        self.assertEqual(sample.conversion_start_us, 600)
        self.assertAlmostEqual(sample.millivolts(3.3), -51.5625)

    def test_fragmentation_boot_noise_and_other_channels(self):
        decoder = BowDecoder()
        data = b"# boot\r\n" + bow_packet(channel=0, seq=0)
        data += bow_packet(seq=1) + bow_packet(kind=1) + bow_packet(kind=3)
        samples = []
        for byte in data:
            samples.extend(decoder.feed(bytes([byte])))
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].seq, 1)
        self.assertEqual(decoder.adc_sequence_gaps, 0)

    def test_corruption_resynchronizes_and_rejects_old_version(self):
        bad = bytearray(bow_packet())
        bad[15] ^= 0x40
        old = bytearray(bow_packet())
        old[2] = 1
        old[-2:] = struct.pack("<H", binascii.crc_hqx(old[:-2], 0xFFFF))
        decoder = BowDecoder()
        samples = decoder.feed(bytes(bad) + old + bow_packet()[:11] + bow_packet())
        self.assertEqual(len(samples), 1)
        self.assertGreaterEqual(decoder.crc_errors, 3)

    def test_fault_saturation_and_invalid_payload_are_rejected(self):
        cases = [dict(status=0xFD60), dict(status=0xFE60), dict(raw=-8388608),
                 dict(raw=8388607), dict(raw=8388608), dict(gain=0), dict(channel=2)]
        for args in cases:
            with self.subTest(args=args):
                self.assertEqual(BowDecoder().feed(bow_packet(**args)), [])

    def test_mcu_wrap_reset_and_adc_sequence_gaps(self):
        decoder = BowDecoder()
        decoder.feed(bow_packet(seq=0xFFFFFFFF, us=0xFFFFFF00))
        sample, = decoder.feed(bow_packet(seq=0, us=744))
        self.assertEqual(sample.epoch, 0)
        self.assertEqual(decoder.adc_sequence_gaps, 0)
        decoder.feed(bow_packet(seq=2, us=1744))
        self.assertEqual(decoder.adc_sequence_gaps, 1)
        sample, = decoder.feed(bow_packet(seq=0, us=400))
        self.assertEqual(sample.epoch, 1)

    def test_nearest_ignores_old_position_and_respects_skew(self):
        stream = BowStream(None)
        sample, = BowDecoder().feed(bow_packet())
        stream.samples.extend([
            replace(sample, received_monotonic_s=9.99),
            replace(sample, seq=2, received_monotonic_s=10.02),
            replace(sample, seq=3, received_monotonic_s=10.04),
        ])
        selected = stream.nearest(10.01, since=10, max_skew=0.02, timeout=0.01)
        self.assertEqual(selected.seq, 2)
        with self.assertRaises(TimeoutError):
            stream.nearest(10.01, since=10, max_skew=0.001, timeout=0.01)
        with self.assertRaises(TimeoutError):
            stream.nearest(10.1, since=10.1, max_skew=0.02, timeout=0.01)

    def test_reader_error_is_propagated(self):
        stream = BowStream(None)
        stream._error = OSError("disconnected")
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            stream.nearest(0, since=0, max_skew=1, timeout=0.01)


class ForceTests(unittest.TestCase):
    def test_manual_crc_vector_and_register_addresses(self):
        # PDF page 2 / Appendix B: literal example, not generated expected CRC.
        self.assertEqual(read_request(1, 0x1000, 2), bytes.fromhex("01 03 10 00 00 02 c0 cb"))
        self.assertEqual(read_request(1)[:6], bytes.fromhex("01 03 00 00 00 0c"))

    def test_all_units_negative_values_and_fragmented_reads(self):
        for unit, (name, scale) in UNITS.items():
            with self.subTest(unit=unit):
                port = ForcePort(force_frame(-2.5, unit))
                sample = ForceGauge(port).read()
                self.assertEqual(sample.unit, name)
                self.assertEqual(sample.value, -2.5)
                self.assertAlmostEqual(sample.newtons, -2.5 * scale)
                self.assertEqual(port.writes, [read_request(1)])
                self.assertEqual(port.timeout, 0.02)
                self.assertLessEqual(sample.request_monotonic_s, sample.midpoint)
                self.assertLessEqual(sample.midpoint, sample.response_monotonic_s)

    def test_gauge_unit_changes_are_read_each_time(self):
        port = ForcePort(force_frame(2, 0))
        gauge = ForceGauge(port)
        self.assertEqual(gauge.read().newtons, 2)
        port.frame = force_frame(2, 1)
        self.assertEqual(gauge.read().newtons, 2000)

    def test_bad_crc_unknown_unit_nonfinite_and_exception(self):
        bad = bytearray(force_frame())
        bad[5] ^= 1
        frames = [bytes(bad), force_frame(unit=99), force_frame(float("nan")),
                  force_frame(float("inf")), with_crc(b"\x01\x83\x02")]
        for frame in frames:
            with self.subTest(frame=frame.hex()):
                with self.assertRaises(ProtocolError):
                    ForceGauge(ForcePort(frame)).read()

    def test_response_identity_size_and_function(self):
        for frame in [force_frame(address=2), with_crc(b"\x01\x04\x02\x00\x00"),
                      with_crc(b"\x01\x03\x02\x00\x00")]:
            with self.subTest(frame=frame.hex()):
                with self.assertRaises(ProtocolError):
                    response_payload(frame, 1, 12)

    def test_timeout_is_bounded_and_restores_serial_timeout(self):
        port = ForcePort(b"")
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            ForceGauge(port, timeout=0.01).read()
        self.assertLess(time.monotonic() - started, 0.2)
        self.assertEqual(port.timeout, 0.02)


class SequenceTests(unittest.TestCase):
    def args(self, output, *extra):
        return parse_args(["--bow-port", "bow", "--force-port", "force", "--output", str(output),
                           "--samples-per-position", "2", "--poll-hz", "1000", "--settle-s", "0",
                           "--bow-length-m", "0.65", *extra])

    def test_complete_sequence_through_both_protocol_readers(self):
        class StreamingPort:
            def __init__(self):
                self.seq = 0

            @property
            def in_waiting(self):
                return 64

            def reset_input_buffer(self):
                pass

            def read(self, size):
                time.sleep(0.001)
                frame = bow_packet(seq=self.seq, channel=0, us=1000 + self.seq * 500)
                frame += bow_packet(seq=self.seq + 1, us=1500 + self.seq * 500)
                self.seq += 2
                return frame

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "pairs.csv"
            args = self.args(output)
            prompts = []
            with Recording(output, {}) as recording, BowStream(StreamingPort()) as bow:
                run_sequence(bow, ForceGauge(ForcePort()), recording, args,
                             prompt=lambda text: prompts.append(text) or "", report=lambda _: None)
            rows = list(csv.DictReader(output.read_text().splitlines()))
            metadata = json.loads(output.with_suffix(".json").read_text())
            self.assertEqual(len(prompts), 10)
            self.assertEqual(len(rows), 20)
            self.assertEqual([float(rows[i]["distance_fraction"]) for i in range(0, 20, 2)],
                             [0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95])
            self.assertEqual(metadata["status"], "complete")
            self.assertEqual(metadata["pairs_recorded"], 20)
            self.assertAlmostEqual(float(rows[-1]["distance_m"]), 0.6175)
            for row in rows:
                self.assertEqual(float(row["force_N"]), 1.25)
                self.assertEqual(int(row["fullbridge_raw24"]), -123456)
                self.assertLessEqual(abs(float(row["pair_skew_ms"])), 50)
                self.assertEqual(len(bytes.fromhex(row["bow_packet_hex"])), 32)

    def test_interrupt_keeps_partial_data_and_marks_aborted(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "partial.csv"
            with self.assertRaises(KeyboardInterrupt):
                with Recording(output, {}) as recording:
                    recording.append({"distance_fraction": "0.05", "force_N": 2})
                    raise KeyboardInterrupt()
            self.assertEqual(len(list(csv.DictReader(output.read_text().splitlines()))), 1)
            metadata = json.loads(output.with_suffix(".json").read_text())
            self.assertEqual(metadata["status"], "aborted")
            self.assertEqual(metadata["pairs_recorded"], 1)

    def test_failure_keeps_rows_and_does_not_mark_position_complete(self):
        class FailingGauge:
            def read(self):
                raise TimeoutError("lost force gauge")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "failure.csv"
            with self.assertRaises(TimeoutError):
                with Recording(output, {}) as recording:
                    run_sequence(BowStream(None), FailingGauge(), recording, self.args(output),
                                 prompt=lambda _: "", report=lambda _: None)
            metadata = json.loads(output.with_suffix(".json").read_text())
            self.assertEqual(metadata["status"], "failed")
            self.assertEqual(metadata["positions_completed"], [])
            self.assertEqual(metadata["pairs_recorded"], 0)

    def test_quit_at_prompt_records_no_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "quit.csv"
            with self.assertRaises(KeyboardInterrupt):
                with Recording(output, {}) as recording:
                    run_sequence(BowStream(None), None, recording, self.args(output),
                                 prompt=lambda _: "q", report=lambda _: None)
            self.assertEqual(json.loads(output.with_suffix(".json").read_text())["status"], "aborted")

    def test_slow_force_transaction_and_reused_bow_sample_fail(self):
        class FixedBow:
            def nearest(self, target, **kwargs):
                sample, = BowDecoder().feed(bow_packet())
                return replace(sample, received_monotonic_s=target)

            def diagnostics(self):
                return {}

        class SlowGauge:
            def read(self):
                sample = ForceGauge(ForcePort()).read()
                return replace(sample, response_monotonic_s=sample.request_monotonic_s + 1)

        for gauge, expected_rows in [(SlowGauge(), 0), (ForceGauge(ForcePort()), 1)]:
            with self.subTest(expected_rows=expected_rows), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "timing.csv"
                with self.assertRaises(TimeoutError):
                    with Recording(output, {}) as recording:
                        run_sequence(FixedBow(), gauge, recording, self.args(output),
                                     prompt=lambda _: "", report=lambda _: None)
                metadata = json.loads(output.with_suffix(".json").read_text())
                self.assertEqual(metadata["status"], "failed")
                self.assertEqual(metadata["pairs_recorded"], expected_rows)
                self.assertEqual(metadata["positions_completed"], [])

    def test_existing_csv_or_metadata_is_never_overwritten(self):
        for suffix in (".csv", ".json"):
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "existing.csv"
                existing = output.with_suffix(suffix)
                existing.write_text("keep me")
                with self.assertRaises(FileExistsError):
                    with Recording(output, {}):
                        pass
                self.assertEqual(existing.read_text(), "keep me")

    def test_cli_rejects_invalid_settings(self):
        for extra in [("--samples-per-position", "0"), ("--poll-hz", "nan"),
                      ("--settle-s", "-1"), ("--force-address", "0"),
                      ("--max-skew-ms", "inf"), ("--avdd", "9"),
                      ("--force-port", "bow")]:
            with self.subTest(extra=extra), patch("sys.stderr"):
                with self.assertRaises(SystemExit):
                    self.args(Path("test.csv"), *extra)


if __name__ == "__main__":
    unittest.main()
