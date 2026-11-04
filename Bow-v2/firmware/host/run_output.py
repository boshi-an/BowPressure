"""Four streaming CSV files; finalize LTC interpolation on orderly close."""
import csv
from pathlib import Path
from export_ltc import merge_directory

COMMON = ['record_type', 'epoch', 'seq',
          'sample_mcu_us', 'mcu_read_us_unwrapped']
FIELDS = {
    'adc': COMMON + ['channel', 'diff_mV', 'raw24', 'gain', 'avdd_V',
                     'adc_start_mcu_us', 'adc_ready_mcu_us', 'adc_filter_delay_us',
                     'sample_time_basis', 'status_hex', 'fault_mask_hex'],
    'imu': COMMON + ['ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
                     'sensor_us16', 'sensor_us_unwrapped', 'imu_clock_scale',
                     'imu_time_segment', 'imu_time_quality'],
    'ltc': COMMON + ['timecode', 'fps', 'locked', 'edge_count', 'decode_errors',
                     'edge_overflows', 'ltc_anchor_mcu_us', 'ltc_anchor_day_us'],
    'mcu': COMMON + ['host_unix_s', 'tx_drops', 'missed_slots', 'max_late_us', 'faults',
                     'alive', 'imu_status_hex', 'int1_pin', 'fifo_count', 'irq_count',
                     'poll_count', 'observed_us', 'peak_count', 'crc_errors',
                     'seq_errors', 'imu_dt_errors', 'invalid_payloads', 'message'],
}
ROUTE = {'ADC': 'adc', 'IMU': 'imu', 'LTC': 'ltc', 'LTC_ANCHOR': 'ltc',
         'STATUS': 'mcu', 'IMU_STATUS': 'mcu', 'LOG': 'mcu'}


class RunOutput:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.handles = {}
        self.writers = {}

    def __enter__(self):
        # A new run directory is required, so an old recording is never mixed in.
        self.directory.mkdir(parents=True, exist_ok=False)
        try:
            for name, fields in FIELDS.items():
                h = (self.directory / (name+'.csv')).open('x', newline='', encoding='utf-8')
                self.handles[name] = h
                w = csv.DictWriter(h, fieldnames=fields, extrasaction='ignore')
                w.writeheader()
                self.writers[name] = w
        except BaseException:
            for h in self.handles.values(): h.close()
            raise
        return self

    def writerow(self, row):
        row = dict(row)
        if row['record_type'] not in ('IMU', 'ADC'):
            row['sample_mcu_us'] = row.get('ltc_anchor_mcu_us', row.get('mcu_read_us_unwrapped', ''))
        self.writers[ROUTE[row['record_type']]].writerow(row)

    def flush(self):
        for h in self.handles.values(): h.flush()

    def __exit__(self, typ, value, traceback):
        for h in self.handles.values(): h.close()
        self.finalize()
        return False

    def finalize(self):
        return merge_directory(self.directory)
