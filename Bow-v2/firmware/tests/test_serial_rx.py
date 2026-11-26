import threading
import unittest
from unittest.mock import patch
from serial_rx import SerialReceiver

class Port:
    def __init__(self, chunks=(), error=None):
        self.chunks = list(chunks)
        self.error = error
        self.empty = threading.Event()
    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0
    def read(self, size):
        if self.chunks:
            return self.chunks.pop(0)
        self.empty.set()
        if self.error:
            raise self.error
        threading.Event().wait(.001)
        return b''

class ReceiverTests(unittest.TestCase):
    def test_stalled_consumer(self):
        chunks = [bytes([i % 256])*1000 for i in range(400)]
        port = Port(chunks)
        with SerialReceiver(port) as rx:
            self.assertTrue(port.empty.wait(2))
            before = rx.snapshot()
            self.assertEqual(before['rx_queue_bytes'], 400000)
            self.assertEqual(before['rx_driver_pending_peak_bytes'], 1000)
            self.assertEqual(before['rx_read_chunks'], 400)
            self.assertEqual(before['rx_dequeued_bytes'], 0)
            self.assertEqual([rx.get() for _ in chunks], [(0,c) for c in chunks])
            after = rx.snapshot()
            self.assertEqual(after['rx_overflow_events'], 0)
            self.assertEqual(after['rx_queue_bytes'], 0)
            self.assertEqual(after['rx_dequeued_bytes'], 400000)
        self.assertFalse(rx.thread.is_alive())
    def test_overflow(self):
        rx = SerialReceiver(Port(), capacity=8)
        rx._enqueue(b'abcdef'); rx._enqueue(b'XYZ')
        self.assertEqual(rx.snapshot()['rx_queue_bytes'], 6)
        self.assertEqual(rx.snapshot()['rx_dropped_bytes'], 3)
        self.assertEqual(rx.get(), (0,b'abcdef'))
        rx._enqueue(b'next')
        self.assertEqual(rx.get(), (1,b'next'))
        self.assertEqual(rx.snapshot()['rx_queue_peak_bytes'], 6)

    def test_ingress_gap_and_queue_timing(self):
        rx = SerialReceiver(Port(), capacity=8)
        self.assertIsNone(rx.snapshot()['rx_read_idle_us'])
        times_ns = [10_000, 40_000, 44_000, 49_000, 50_000, 56_000, 59_000]
        with patch('serial_rx.monotonic_ns', side_effect=times_ns):
            rx._enqueue(b'abcdef', pending=6)
            rx._enqueue(b'XYZ', pending=8192)
            first = rx.snapshot(reset_interval=True)
            self.assertEqual(first['rx_read_chunks'], 2)
            self.assertEqual(first['rx_received_bytes'], 9)
            self.assertEqual(first['rx_last_read_monotonic_ns'], 40_000)
            self.assertEqual(first['rx_read_idle_us'], 4)
            self.assertEqual(first['rx_max_read_gap_at_monotonic_ns'], 40_000)
            self.assertEqual(first['rx_interval_max_read_gap_us'], 30)
            self.assertEqual(first['rx_interval_driver_pending_peak_bytes'], 8192)
            self.assertEqual(first['rx_interval_queue_max_residence_us'], 0)
            self.assertEqual(rx.overflow_events(), 1)

            self.assertEqual(rx.get(), (0, b'abcdef'))
            rx._enqueue(b'next', pending=4)
            self.assertEqual(rx.get(), (1, b'next'))
            second = rx.snapshot()
            self.assertEqual(second['rx_max_read_gap_us'], 30)
            self.assertEqual(second['rx_interval_max_read_gap_us'], 10)
            self.assertEqual(second['rx_driver_pending_peak_bytes'], 8192)
            self.assertEqual(second['rx_interval_driver_pending_peak_bytes'], 4)
            self.assertEqual(second['rx_queue_max_residence_us'], 39)
            self.assertEqual(second['rx_interval_queue_max_residence_us'], 39)
            self.assertEqual(second['rx_dequeued_bytes'], 10)

    def test_disconnect(self):
        port = Port([b'abc'], OSError('disconnected'))
        with SerialReceiver(port) as rx:
            self.assertTrue(port.empty.wait(2))
            self.assertEqual(rx.get(), (0,b'abc'))
            with self.assertRaisesRegex(OSError, 'disconnected'): rx.get()
    def test_idle_close(self):
        with SerialReceiver(Port()) as rx:
            self.assertEqual(rx.get(.005), (None,b''))
        self.assertFalse(rx.thread.is_alive())
