"""Causal FIFO clock mapping. Estimated time, not hardware synchronization."""
from collections import deque
from statistics import median

FIELDS = ['sensor_us_unwrapped', 'imu_sample_mcu_us', 'imu_read_delay_us',
          'imu_clock_scale', 'imu_time_segment', 'imu_time_quality']


class ImuClock:
    def __init__(self, rate_hz=3200):
        period = 1_000_000 / rate_hz
        self.min_dt, self.max_dt = .8 * period, 1.2 * period
        self.segment = -1
        self.last = None

    def valid_interval(self, dt):
        return self.min_dt <= dt <= self.max_dt

    def update(self, sensor, read, seq, epoch):
        previous = self.last
        dt = (sensor - previous[0]) & 0xffff if previous else 0
        # Do not guess how many 16-bit wraps occurred across missing frames.
        restart = (previous is None or epoch != previous[3] or
                   (seq - previous[2]) & 0xffffffff != 1 or
                   not self.valid_interval(dt) or read < previous[1])
        if restart:
            self.segment += 1
            self.x = sensor
            self.origin = self.x
            self.anchors = deque(maxlen=32)
            self.block_start = self.x
            self.best = (self.x, read)
            self.scale = 1.0
            self.offset = read - self.x
            self.mapped = float(read)
        else:
            self.x += dt
            if self.x - self.block_start >= 1_000_000:
                self.anchors.append(self.best)
                self.block_start = self.x
                self.best = (self.x, read)
                # Low-delay anchors suppress FIFO batching. Long baselines
                # reduce the influence of scheduling jitter on frequency fit.
                slopes = [(b[1]-a[1])/(b[0]-a[0])
                          for i, a in enumerate(self.anchors)
                          for b in list(self.anchors)[i+1:] if b[0]-a[0] >= 4_000_000]
                if len(self.anchors) >= 8 and slopes:
                    self.scale = min(1.05, max(.95, median(slopes)))
                self.offset = min(y-self.scale*x for x, y in self.anchors)
            if read-self.scale*self.x < self.best[1]-self.scale*self.best[0]:
                self.best = (self.x, read)
            # Track a new lower-delay observation immediately in the model,
            # but slew the published clock to avoid timestamp steps/backtracking.
            self.offset = min(self.offset, read-self.scale*self.x)
            prediction = self.mapped + self.scale*dt
            error = self.scale*self.x + self.offset - prediction
            correction = max(-.02*dt, min(.02*dt, error))
            self.mapped = min(float(read), max(self.mapped, prediction+correction))
        self.last = (sensor, read, seq, epoch)
        return dict(sensor_us_unwrapped=self.x, imu_sample_mcu_us=self.mapped,
                    imu_read_delay_us=read-self.mapped, imu_clock_scale=self.scale,
                    imu_time_segment=self.segment,
                    imu_time_quality='tracking' if len(self.anchors) >= 8 else 'warmup')
