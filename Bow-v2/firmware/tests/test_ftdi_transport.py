from collections import deque
from contextlib import ExitStack
import csv
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from ftdi_usb import FTDI_STATUS_FIELDS
from raw_capture import RawCapture
from read import main
from serial_rx import SerialReceiver
import serial_transport
from test_read_fast import packet


DEVICE = '/dev/cu.usbserial-bow'


def port_info(device=DEVICE, vid=0x0403, pid=0x6001, serial_number='BOW123'):
    return SimpleNamespace(device=device, vid=vid, pid=pid, serial_number=serial_number)


class TransportTests(unittest.TestCase):
    def setUp(self):
        mocks = ExitStack()
        self.addCleanup(mocks.close)
        self.ports = mocks.enter_context(patch('serial.tools.list_ports.comports'))
        self.ports.return_value = [
            port_info(device='/dev/cu.usbserial-other', serial_number='OTHER123'), port_info(),
        ]
        self.usb = mocks.enter_context(patch.object(serial_transport, 'FtdiUsb'))
        self.usb_handle = self.usb.return_value.__enter__.return_value
        self.usb_handle.transport_name = 'ftdi-usb'
        self.vcp = mocks.enter_context(patch('serial.Serial'))
        self.vcp.return_value.__enter__.return_value = self.vcp.return_value

    def test_auto_uses_selected_ft232r_at_supported_baud_rates(self):
        with patch.object(sys, 'platform', 'darwin'):
            for device, baud in ((DEVICE, 1_500_000), ('/dev/tty.usbserial-bow', 3_000_000)):
                with self.subTest(device=device, baud=baud):
                    with serial_transport.open_transport(device, baud) as source:
                        self.assertIs(source, self.usb_handle)
                        self.assertEqual(source.transport_name, 'ftdi-usb')
                    self.usb.assert_called_with('BOW123', baud)
        self.vcp.assert_not_called()
        self.assertEqual(self.usb.return_value.__exit__.call_count, 2)

    def test_auto_keeps_serial_for_unsupported_ports(self):
        cases = [
            ('linux', port_info(), 3_000_000),
            ('darwin', port_info(pid=0x6014), 3_000_000),
            ('darwin', port_info(serial_number=None), 3_000_000),
            ('darwin', port_info(), 115_200),
            ('darwin', port_info(device='/dev/cu.usbserial-other'), 3_000_000),
        ]
        for platform, info, baud in cases:
            with self.subTest(platform=platform, info=info, baud=baud):
                self.ports.return_value = [info]
                with patch.object(sys, 'platform', platform):
                    with serial_transport.open_transport(DEVICE, baud) as source:
                        self.assertEqual(source.transport_name, 'serial')
        self.usb.assert_not_called()
        self.assertEqual(self.vcp.call_count, len(cases))

    def test_explicit_serial_bypasses_usb_matching(self):
        with patch.object(sys, 'platform', 'darwin'):
            with serial_transport.open_transport(DEVICE, 3_000_000, 'serial') as source:
                self.assertEqual(source.transport_name, 'serial')
        self.ports.assert_not_called()
        self.usb.assert_not_called()
        self.vcp.assert_called_once()

    def test_forced_direct_rejects_unsupported_device_platform_or_rate(self):
        for platform, device, baud in (
            ('linux', DEVICE, 3_000_000),
            ('darwin', '/dev/cu.missing', 3_000_000),
            ('darwin', DEVICE, 115_200),
        ):
            with self.subTest(platform=platform, device=device, baud=baud):
                with patch.object(sys, 'platform', platform), self.assertRaises(OSError):
                    with serial_transport.open_transport(device, baud, 'ftdi-usb'):
                        self.fail('An invalid direct USB selection opened a transport')
        self.usb.assert_not_called()
        self.vcp.assert_not_called()

    def test_open_failure_for_matched_adapter_is_visible(self):
        self.usb.return_value.__enter__.side_effect = OSError('USB interface is busy')
        with patch.object(sys, 'platform', 'darwin'), self.assertRaisesRegex(OSError, 'busy'):
            with serial_transport.open_transport(DEVICE, 3_000_000):
                self.fail('A failed direct USB claim opened a transport')
        self.vcp.assert_not_called()


class ChunkSource:
    transport_name = 'ftdi-usb'

    def __init__(self, chunks):
        self.chunks = deque(chunks)
        self.empty = threading.Event()

    def read_chunk(self):
        if self.chunks:
            return self.chunks.popleft()
        self.empty.set()
        threading.Event().wait(.001)
        return b''

    def status(self):
        return dict.fromkeys(FTDI_STATUS_FIELDS, 0) | {'ftdi_overrun_packets': 2}


class DirectReceiverTests(unittest.TestCase):
    def test_direct_chunks_reach_queue_and_raw_capture(self):
        source = ChunkSource([b'hello', b'', b'world'])
        raw = RawCapture(max_bytes=32, max_chunks=8)
        with SerialReceiver(source, raw_capture=raw) as receiver:
            self.assertTrue(source.empty.wait(1))
            self.assertEqual(receiver.get(), (0, b'hello'))
            self.assertEqual(receiver.get(), (0, b'world'))
            first = receiver.snapshot(reset_interval=True)
            second = receiver.snapshot()
            self.assertEqual(first['rx_received_bytes'], 10)
            self.assertEqual(first['rx_overflow_events'], 0)
            self.assertEqual(first['ftdi_overrun_packets'], 2)
            self.assertEqual(second['ftdi_overrun_packets'], 2)
        self.assertFalse(receiver.thread.is_alive())
        with tempfile.TemporaryDirectory() as directory:
            raw.dump(Path(directory))
            self.assertEqual((Path(directory) / 'serial.bin').read_bytes(), b'helloworld')

    def test_direct_receiver_status_reaches_recorder_csv(self):
        wire = packet(3, 1, 100, b'')
        source = ChunkSource([wire])
        states = iter(['recording', 'stopping'])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'run'
            with patch.object(sys, 'platform', 'darwin'), \
                    patch('serial.tools.list_ports.comports', return_value=[port_info()]), \
                    patch.object(serial_transport, 'FtdiUsb') as usb:
                usb.return_value.__enter__.return_value = source
                self.assertEqual(main(['--port', DEVICE, '--output', str(output),
                                       '--diagnostics', '--raw-capture'],
                                      control=lambda: next(states), report=lambda *args: None), 0)
            with (output / 'mcu.csv').open(newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(any(row['record_type'] == 'STATUS' for row in rows))
            self.assertTrue(any(row['record_type'] == 'LOG' and row['ftdi_overrun_packets'] == '2'
                                for row in rows))
            self.assertEqual((output / 'serial.bin').read_bytes(), wire)
