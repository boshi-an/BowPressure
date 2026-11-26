"""Host timing and packet-gap diagnostics for the bow serial stream."""
import time


PACKET_NAMES = {1: 'IMU', 2: 'ADC', 3: 'STATUS', 4: 'LTC', 5: 'IMU_STATUS', 6: 'LTC_ANCHOR', 7: 'DIAG'}
MCU_DEBUG_FIELDS = (
    'boot_reset_flags', 'main_loop_max_gap_us', 'uart_dma_completed_bytes',
    'tx_ring_peak_bytes', 'ltc_isr_edges',
)


def format_mcu_debug(packet):
    return (f"# MCU DEBUG boot_flags=0x{packet['boot_reset_flags']:08X}"
            f" loop_gap_max_us={packet['main_loop_max_gap_us']}"
            f" dma_completed={packet['uart_dma_completed_bytes']} B"
            f" tx_ring_peak={packet['tx_ring_peak_bytes']} B"
            f" ltc_isr_edges={packet['ltc_isr_edges']}")


PIPE_FIELDS = [
    'host_monotonic_ns', 'diag_interval_us', 'decoder_discarded_bytes',
    'decoder_header_errors', 'decoder_pending_bytes', 'decoded_packets', 'saved_packets',
    'decode_us', 'process_us', 'csv_write_us', 'csv_flush_us', 'csv_flush_max_us',
    'gap_stream', 'gap_from_seq', 'gap_missing_packets', 'gap_from_mcu_us', 'gap_mcu_us',
]


class CaptureDiagnostics:
    def __init__(self):
        self.last_report_ns = time.monotonic_ns()
        self.decoded = self.saved = 0
        self.decode_ns = self.process_ns = self.write_ns = self.flush_ns = self.max_flush_ns = 0
        self.previous = {}
        self.last_sensor_us = None

    def observe(self, packet, epoch, mcu_us, previous_mcu_us):
        kind = packet['type']
        seq = packet['seq']
        previous = self.previous.get(kind)
        self.previous[kind] = seq, mcu_us
        if (self.last_sensor_us is not None and previous_mcu_us - self.last_sensor_us < 100000
                and mcu_us - previous_mcu_us >= 20000):
            yield dict(epoch=epoch, gap_stream='ALL', gap_from_mcu_us=previous_mcu_us,
                       gap_mcu_us=mcu_us-previous_mcu_us, mcu_read_us_unwrapped=mcu_us)
        if kind in (1, 2):
            self.last_sensor_us = mcu_us
        if previous is not None:
            old_seq, old_us = previous
            delta = (seq - old_seq) & 0xFFFFFFFF
            if delta != 1 or (kind in (1, 2) and mcu_us - old_us >= 10000):
                yield dict(epoch=epoch, seq=seq, gap_stream=PACKET_NAMES[kind], gap_from_seq=old_seq,
                           gap_missing_packets=delta-1 if 1 <= delta < 0x80000000 else '',
                           gap_from_mcu_us=old_us, gap_mcu_us=mcu_us-old_us,
                           mcu_read_us_unwrapped=mcu_us)

    def reset_epoch(self):
        self.previous.clear()
        self.last_sensor_us = None

    def snapshot(self, decoder, now_ns):
        stats = dict(
            host_monotonic_ns=now_ns, diag_interval_us=(now_ns - self.last_report_ns) // 1000,
            decoder_discarded_bytes=decoder.discarded, decoder_header_errors=decoder.header_errors,
            decoder_pending_bytes=len(decoder.buffer), decoded_packets=self.decoded,
            saved_packets=self.saved, decode_us=self.decode_ns // 1000,
            process_us=self.process_ns // 1000, csv_write_us=self.write_ns // 1000,
            csv_flush_us=self.flush_ns // 1000, csv_flush_max_us=self.max_flush_ns // 1000,
        )
        self.last_report_ns = now_ns
        self.decoded = self.saved = 0
        self.decode_ns = self.process_ns = self.write_ns = self.flush_ns = self.max_flush_ns = 0
        return stats
