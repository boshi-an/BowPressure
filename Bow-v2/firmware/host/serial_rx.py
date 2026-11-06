"""Bounded byte queue: serial I/O never waits for CSV, plotting or decoding."""
from collections import deque
import threading

RX_FIELDS = ['rx_queue_bytes', 'rx_queue_capacity', 'rx_queue_peak_bytes',
             'rx_received_bytes', 'rx_overflow_events', 'rx_dropped_bytes']


class SerialReceiver:
    def __init__(self, port, capacity=1024 * 1024):
        if capacity <= 0:
            raise ValueError('capacity must be positive')
        self.port, self.capacity = port, capacity
        self.condition = threading.Condition()
        self.queue = deque()
        self.size = self.peak = self.received = self.overflows = self.dropped = 0
        self.error = None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._run, name='serial-receive', daemon=False)

    def __enter__(self):
        self.thread.start()
        return self

    def _enqueue(self, chunk):
        with self.condition:
            self.received += len(chunk)
            if self.size + len(chunk) > self.capacity:
                # Keep draining the device. Mark the discontinuity on the next
                # accepted chunk so decoder fragments cannot bridge lost bytes.
                self.overflows += 1
                self.dropped += len(chunk)
            else:
                self.queue.append((self.overflows, chunk))
                self.size += len(chunk)
                self.peak = max(self.peak, self.size)
            self.condition.notify_all()

    def _run(self):
        try:
            while not self.stopped.is_set():
                chunk = self.port.read(min(4096, max(1, self.port.in_waiting)))
                if chunk:
                    self._enqueue(chunk)
        except BaseException as exc:
            with self.condition:
                self.error = exc
                self.condition.notify_all()

    def get(self, timeout=.05):
        with self.condition:
            self.condition.wait_for(lambda: self.queue or self.error or self.stopped.is_set(), timeout)
            if self.queue:
                generation, chunk = self.queue.popleft()
                self.size -= len(chunk)
                return generation, chunk
            if self.error:
                raise self.error
            return None, b''

    def snapshot(self):
        with self.condition:
            return dict(zip(RX_FIELDS, (self.size, self.capacity, self.peak,
                                       self.received, self.overflows, self.dropped)))

    def __exit__(self, *args):
        self.stopped.set()
        # The port uses a 50 ms read timeout. Join before its owner closes it.
        self.thread.join()
        with self.condition:
            self.condition.notify_all()
