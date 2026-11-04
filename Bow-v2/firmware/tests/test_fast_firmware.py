"""Native tests of production scheduling/ring code; registers are mocks."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which('cc'), 'native C compiler required')
class FirmwareLogicTests(unittest.TestCase):
    def test_scheduler_and_dma_ring(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            for name in ('test_fast_adc_irq', 'test_fast_adc', 'test_fast_tx', 'test_fast_timer', 'test_fast_board', 'test_fast_imu'):
                with self.subTest(name=name):
                    exe = Path(directory) / name
                    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                    str(root / 'tests' / (name + '.c')), '-o', str(exe)],
                                   check=True, capture_output=True, text=True)
                    subprocess.run([str(exe)], check=True, capture_output=True, timeout=5)


if __name__ == '__main__':
    unittest.main()
