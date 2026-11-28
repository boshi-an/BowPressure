from collections import deque
import gc
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import weakref

from ftdi_usb import FTDI_STATUS_FIELDS, FtdiUsb


class UsbError(Exception):
    pass


class Transfer:
    def __init__(self):
        self.submissions = 0

    def setBulk(self, endpoint, size, callback, timeout):
        self.callback = callback

    def submit(self):
        self.submissions += 1

    def complete(self, data):
        self.data = data
        self.callback(self)

    def getStatus(self):
        return 0

    def getBuffer(self):
        return self.data

    def getActualLength(self):
        return len(self.data)


class Handle:
    def __init__(self, transfers, closed, setup):
        self.transfers = transfers
        self.setup = setup
        # python-libusb1 exposes close as a finalizer that does not own its handle.
        self.close = weakref.finalize(self, closed.append, 'handle')

    def claimInterface(self, interface):
        pass

    def controlWrite(self, request_type, request, value, index, data, timeout):
        self.setup.append((request_type, request, value, index, data))

    def getTransfer(self):
        if not self.transfers:
            self.setup.append('bulk transfer')
        transfer = Transfer()
        self.transfers.append(transfer)
        return transfer


class UsbContext:
    def __init__(self):
        self.events = deque()
        self.transfers = []
        self.closed = []
        self.setup = []

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.closed.append('context')

    def getDeviceList(self, skip_on_error):
        return [SimpleNamespace(getVendorID=lambda: 0x0403, getProductID=lambda: 0x6001,
                                getSerialNumber=lambda: 'BOW123', open=self.open_handle)]

    def open_handle(self):
        handle = Handle(self.transfers, self.closed, self.setup)
        self.handle = weakref.ref(handle)
        return handle

    def handleEventsTimeout(self, timeout):
        if self.events:
            self.events.popleft()()


class FtdiUsbTests(unittest.TestCase):
    def setUp(self):
        self.context = UsbContext()
        usb = SimpleNamespace(USBContext=lambda: self.context, USBError=UsbError,
                              USBErrorInterrupted=UsbError, TRANSFER_COMPLETED=0)
        load = patch('ftdi_usb._load_usb', return_value=usb)
        load.start()
        self.addCleanup(load.stop)

    def test_open_context_keeps_handle_alive_until_close(self):
        with FtdiUsb('BOW123', 3_000_000):
            gc.collect()
            self.assertIsNotNone(self.context.handle())
            self.assertEqual(self.context.closed, [])
        self.assertEqual(self.context.closed, ['handle', 'context'])

    def test_purges_stale_input_before_arming_bulk_reads(self):
        with FtdiUsb('BOW123', 3_000_000):
            self.assertEqual(self.context.setup[-2:], [
                (0x40, 0, 2, 0, b''),
                'bulk transfer',
            ])

    def test_orders_completions_strips_status_and_counts_uart_errors(self):
        with FtdiUsb('BOW123', 3_000_000) as source:
            first, second = self.context.transfers[:2]
            self.context.events.extend([
                lambda: second.complete(b'\x01\x0adef'),
                lambda: first.complete(b'\x01\x02' + b'a' * 62 + b'\x01\x04'),
            ])
            self.assertEqual(source.read_chunk(), b'')
            self.assertEqual(source.status(), dict.fromkeys(FTDI_STATUS_FIELDS, 0))
            self.assertEqual(second.submissions, 1)

            self.assertEqual(source.read_chunk(), b'a' * 62 + b'def')
            self.assertEqual(source.status(), dict.fromkeys(FTDI_STATUS_FIELDS, 0) | {
                'ftdi_overrun_packets': 2,
                'ftdi_parity_error_packets': 1,
                'ftdi_framing_error_packets': 1,
            })
            self.assertEqual((first.submissions, second.submissions), (2, 2))
