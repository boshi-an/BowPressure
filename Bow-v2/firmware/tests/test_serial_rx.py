import threading
import unittest
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
            self.assertEqual(rx.snapshot()['rx_queue_bytes'], 400000)
            self.assertEqual([rx.get() for _ in chunks], [(0,c) for c in chunks])
            self.assertEqual(rx.snapshot()['rx_overflow_events'], 0)
            self.assertEqual(rx.snapshot()['rx_queue_bytes'], 0)
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
