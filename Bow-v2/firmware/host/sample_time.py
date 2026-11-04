"""ADC effective sample-time estimate for Mode3, OSR256 (sinc4(32)+sinc1(8))."""
# Linear-phase FIR group delay, in modulator periods:
# [4*(32-1) + 32*(8-1)]/2 = 174. fMOD = 1.024 MHz.
ADC_FILTER_DELAY_US = 174 / 1.024
FIELDS = ['sample_mcu_us', 'sample_time_basis', 'adc_start_mcu_us',
          'adc_ready_mcu_us', 'adc_filter_delay_us']


def adc_time(read32, read_unwrapped, start32, age, delay=ADC_FILTER_DELAY_US):
    elapsed = (read32-start32) & 0xffffffff
    # Reject impossible time relationships rather than creating a fake sample time.
    if not 0 <= age <= elapsed <= 20000 or delay < 0:
        return dict(sample_time_basis='invalid_adc_timing')
    start = read_unwrapped-elapsed
    ready = start+age
    if ready-delay < start:
        return dict(adc_start_mcu_us=start, adc_ready_mcu_us=ready,
                    sample_time_basis='invalid_adc_timing')
    return dict(sample_mcu_us=ready-delay, sample_time_basis='adc_filter_delay_estimate',
                adc_start_mcu_us=start, adc_ready_mcu_us=ready, adc_filter_delay_us=delay)
