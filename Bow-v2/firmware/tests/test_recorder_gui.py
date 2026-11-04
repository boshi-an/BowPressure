import csv
import io
import struct
import tempfile
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
from read import main
from recorder_gui import Recorder
from test_read_fast import packet


class GuiTests(unittest.TestCase):
    def test_pause_drain_resume_stop(self):
        states = iter(['recording', 'paused', 'recording', 'stopping'])
        class Port:
            in_waiting = 32
            seq = 0
            closed = False
            def open(self): pass
            def close(self): self.closed = True
            def read(self, n):
                result = packet(3, self.seq, 100+self.seq*100,
                                struct.pack('<IIIIH', 0,0,0,0,3))
                self.seq += 1
                return result
        port = Port()
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)/'run'
            with patch('read.serial.Serial', return_value=port):
                result = main(['--port','fake','--output',str(root)],
                              control=lambda: next(states), report=lambda *a: None)
            self.assertEqual(result, 0)
            self.assertEqual(port.seq, 3)  # All three packets drained, including paused one.
            self.assertTrue(port.closed)
            with (root/'mcu.csv').open() as f: rows = list(csv.DictReader(f))
            self.assertEqual([r['seq'] for r in rows], ['0', '2'])
            self.assertEqual(rows[-1]['seq_errors'], '0')
            self.assertTrue((root/'merged.csv').exists())

    def test_state_machine_and_output_validation(self):
        r = Recorder()
        r.shutting_down = True  # This test stops the worker without auto-preview.
        with self.assertRaises(ValueError): r.transition('pause')
        device = type('Device', (), {'device': '/dev/fake'})()
        finish = threading.Event()
        def fake_record(args, control, report, on_sample, output_provider):
            finish.wait(3)
            return 0
        with tempfile.TemporaryDirectory() as d, patch('recorder_gui.list_ports.comports', return_value=[device]), \
                patch('recorder_gui.record', side_effect=fake_record):
            with self.assertRaises(ValueError): r.start('/dev/fake', d, '../escape', 1500000)
            r.start('/dev/fake', d, 'run1', 1500000)
            try:
                with self.assertRaises(ValueError): r.start('/dev/fake', d, 'run2', 1500000)
                r.transition('pause');self.assertEqual(r.control(), 'paused')
                r.transition('resume');self.assertEqual(r.control(), 'recording')
                r.transition('stop');self.assertEqual(r.control(), 'stopping')
            finally:
                finish.set();r.thread.join(3)
            self.assertEqual(r.control(), 'finished')

    def test_preview_envelope_bound_and_reset(self):
        r = Recorder()
        def adc(t, value, epoch=0):
            r.sample(dict(record_type='ADC', epoch=epoch, sample_mcu_us=t,
                          mcu_read_us_unwrapped=t, channel='H', diff_mV=value))
        adc(100000, 1);adc(101000, -10);adc(102000, 8)
        p = r.chart()['series']['H'][0]
        self.assertEqual(p[1:], [[-10], [8], [8]])
        for i in range(1000): adc(120000+i*20000, i)
        self.assertEqual(len(r.chart()['series']['H']), 500)
        self.assertFalse(r.chart()['stale']['H'])
        self.assertTrue(r.chart()['stale']['ACC'])
        adc(1000, 2, epoch=1)
        self.assertEqual(len(r.chart()['series']['H']), 1)
        self.assertEqual(r.chart()['epoch'], 1)

    def test_preview_then_record_without_reopening_serial(self):
        class Port:
            in_waiting = 32
            seq = 0
            opens = 0
            def open(self): self.opens += 1
            def close(self): pass
            def read(self, n):
                result = packet(2, self.seq, 1500+self.seq*1000,
                                struct.pack('<iHBBII', 123,0xff00,0,32,1000+self.seq*1000,383))
                self.seq += 1
                return result
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)/'run'
            port = Port(); seen=[]
            states = iter(['previewing', 'recording', 'stopping'])
            def control():
                state = next(states)
                if state == 'recording': self.assertFalse(root.exists())
                return state
            with patch('read.serial.Serial', return_value=port):
                self.assertEqual(main(['--port','fake'], control=control,
                    output_provider=lambda: str(root), on_sample=seen.append, report=lambda *a: None), 0)
            self.assertEqual(port.opens, 1)
            self.assertEqual(len(seen), 2)
            with (root/'adc.csv').open() as f: rows=list(csv.DictReader(f))
            self.assertEqual([r['seq'] for r in rows], ['1'])
            self.assertTrue((root/'merged.csv').exists())

    def test_preview_only_creates_no_files(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)/'unused'
            states = iter(['previewing', 'stopping'])
            class Port:
                in_waiting=0
                def open(self): pass
                def close(self): pass
                def read(self, n): return b''
            with patch('read.serial.Serial', return_value=Port()):
                main(['--port','fake'], control=lambda: next(states),
                     output_provider=lambda: str(root), report=lambda *a: None)
            self.assertFalse(root.exists())

    def test_recording_background_excludes_pause(self):
        r = Recorder()
        for t, saving in [(1000,False),(2000,True),(3000,True),(4000,False),(5000,True)]:
            r.sample(dict(record_type='ADC', epoch=0, sample_mcu_us=t,
                          mcu_read_us_unwrapped=t, channel='H', diff_mV=1, recording=saving))
        self.assertEqual(r.chart()['recorded'], [[.001,.002],[.004,.004]])

    def test_recording_returns_to_preview(self):
        r = Recorder()
        r.output = '/tmp/example-recording'
        r.connection = ('/dev/fake',1500000)
        r.state = 'stopping'
        with patch('recorder_gui.record', return_value=0), patch.object(r, 'start') as start:
            r._run([])
        start.assert_called_once_with('/dev/fake', '', '', 1500000, preview_only=True, preserve=True)
        self.assertEqual(r.last_output, '/tmp/example-recording')

    def test_preview_connection_switch(self):
        r = Recorder()
        r.state = 'previewing'
        r.connection = ('/dev/old', 1500000)
        ports = [type('Port', (), {'device': '/dev/new'})()]
        stopped = threading.Event()
        def reader():
            while r.control() != 'stopping':
                stopped.wait(.001)
            with r.lock: r.state = 'idle'
        r.thread = threading.Thread(target=reader)
        r.thread.start()
        try:
            with patch('recorder_gui.list_ports.comports', return_value=ports), patch.object(r, 'start') as start:
                r.switch_preview('/dev/new', 115200)
                self.assertFalse(r.thread.is_alive())
                start.assert_called_once_with('/dev/new', '', '', 115200, preview_only=True)
            r.state = 'recording'
            with self.assertRaises(ValueError): r.switch_preview('/dev/new', 115200)
        finally:
            r.state = 'stopping'
            r.thread.join(1)
