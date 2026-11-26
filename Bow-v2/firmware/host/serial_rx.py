"""Bounded byte queue: serial I/O never waits for CSV, plotting or decoding."""
from collections import deque
import threading
from time import monotonic_ns

RX_FIELDS = ['rx_queue_bytes', 'rx_queue_capacity', 'rx_queue_peak_bytes',
             'rx_received_bytes', 'rx_overflow_events', 'rx_dropped_bytes',
             'rx_read_chunks', 'rx_last_read_monotonic_ns', 'rx_read_idle_us',
             'rx_max_read_gap_us', 'rx_max_read_gap_at_monotonic_ns',
             'rx_driver_pending_peak_bytes', 'rx_dequeued_bytes',
             'rx_queue_max_residence_us', 'rx_interval_max_read_gap_us',
             'rx_interval_driver_pending_peak_bytes', 'rx_interval_queue_max_residence_us']


class SerialReceiver:
    def __init__(self, port, capacity=1024 * 1024, *, raw_capture=None):
        if capacity <= 0:
            raise ValueError('capacity must be positive')
        self.port, self.capacity = port, capacity
        self.condition = threading.Condition()
        self.queue = deque()
        self.size = self.peak = self.received = self.overflows = self.dropped = 0
        self.read_chunks = self.dequeued_bytes = 0
        self.last_read_ns = self.max_gap_ns = self.max_gap_at_ns = 0
        self.max_pending = self.max_residence_ns = 0
        self.interval_gap_ns = self.interval_pending = self.interval_residence_ns = 0
        self.error = None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._run, name='serial-receive', daemon=False)
        self.raw_capture = None
        if raw_capture is not None:
            self.set_raw_capture(raw_capture)

    def __enter__(self):
        self.thread.start()
        return self

    def set_raw_capture(self, capture):
        """Tap subsequent reads, before the consumer queue can discard them."""
        with self.condition:
            if self.raw_capture is not None:
                raise ValueError('receiver already has a raw capture')
            capture.attach(self.received, self.read_chunks)
            self.raw_capture = capture

    def _observe_pending(self, pending):
        """Record the driver's existing in_waiting sample under the queue lock."""
        self.max_pending = max(self.max_pending, pending)
        self.interval_pending = max(self.interval_pending, pending)

    def _enqueue(self, chunk, pending=0):
        read_ns = monotonic_ns()
        with self.condition:
            self._observe_pending(pending)
            if self.read_chunks:
                gap = read_ns - self.last_read_ns
                if gap > self.max_gap_ns:
                    self.max_gap_ns, self.max_gap_at_ns = gap, read_ns
                self.interval_gap_ns = max(self.interval_gap_ns, gap)
            self.last_read_ns = read_ns
            self.read_chunks += 1
            self.received += len(chunk)
            if self.raw_capture is not None:
                self.raw_capture.append(chunk, read_ns)
            if self.size + len(chunk) > self.capacity:
                # Keep draining the device. Mark the discontinuity on the next
                # accepted chunk so decoder fragments cannot bridge lost bytes.
                self.overflows += 1
                self.dropped += len(chunk)
            else:
                self.queue.append((self.overflows, chunk, read_ns))
                self.size += len(chunk)
                self.peak = max(self.peak, self.size)
            self.condition.notify_all()

    def _run(self):
        pending = 0
        try:
            while not self.stopped.is_set():
                pending = 0
                pending = self.port.in_waiting
                chunk = self.port.read(min(4096, max(1, pending)))
                if chunk:
                    self._enqueue(chunk, pending)
                elif pending:
                    with self.condition:
                        self._observe_pending(pending)
        except BaseException as exc:
            with self.condition:
                self._observe_pending(pending)
                self.error = exc
                self.condition.notify_all()

    def get(self, timeout=.05):
        with self.condition:
            self.condition.wait_for(lambda: self.queue or self.error or self.stopped.is_set(), timeout)
            if self.queue:
                generation, chunk, read_ns = self.queue.popleft()
                residence = monotonic_ns() - read_ns
                self.max_residence_ns = max(self.max_residence_ns, residence)
                self.interval_residence_ns = max(self.interval_residence_ns, residence)
                self.size -= len(chunk)
                self.dequeued_bytes += len(chunk)
                return generation, chunk
            if self.error:
                raise self.error
            return None, b''

    def overflow_events(self):
        with self.condition:
            return self.overflows

    def snapshot(self, *, reset_interval=False):
        with self.condition:
            idle_us = (monotonic_ns() - self.last_read_ns) // 1000 if self.read_chunks else None
            values = (self.size, self.capacity, self.peak, self.received,
                      self.overflows, self.dropped, self.read_chunks, self.last_read_ns,
                      idle_us, self.max_gap_ns // 1000, self.max_gap_at_ns,
                      self.max_pending, self.dequeued_bytes, self.max_residence_ns // 1000,
                      self.interval_gap_ns // 1000, self.interval_pending, self.interval_residence_ns // 1000)
            if reset_interval:
                self.interval_gap_ns = self.interval_pending = self.interval_residence_ns = 0
            return dict(zip(RX_FIELDS, values))

    def __exit__(self, *args):
        self.stopped.set()
        # The port uses a 50 ms read timeout. Join before its owner closes it.
        self.thread.join()
        with self.condition:
            self.condition.notify_all()
