#!/usr/bin/env python3
"""Local web recording control. Run with the stm32 Python environment."""
import argparse
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import subprocess
import sys
import threading
import time
import webbrowser
from serial.tools import list_ports
from read import main as record

ROOT = Path(__file__).resolve().parents[1]


class Recorder:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = 'idle'
        self.logs = deque(maxlen=120)
        self.thread = None
        self.output = ''
        self.started = None
        self.connection = None
        self.shutting_down = False
        self.last_output = ''
        self.recorded = deque(maxlen=500)
        self.was_recording = False
        self.preview_epoch = None
        self.preview_origin = None
        self.preview = {k: deque(maxlen=500) for k in ('ACC', 'GYRO', 'H', 'F')}
        self.preview_arrival = {}

    def report(self, *args, **kwargs):
        with self.lock:
            self.logs.append(' '.join(map(str, args)))

    def control(self):
        with self.lock:
            return self.state

    def sample(self, row):
        """20ms min/max buckets for display only; CSV samples remain untouched."""
        epoch = row['epoch']
        stamp = row.get('sample_mcu_us', row['mcu_read_us_unwrapped'])
        if row['record_type'] == 'IMU':
            groups = {'ACC': [row[k] for k in ('ax_g', 'ay_g', 'az_g')],
                      'GYRO': [row[k] for k in ('gx_dps', 'gy_dps', 'gz_dps')]}
        else:
            groups = {row['channel']: [row['diff_mV']]}
        with self.lock:
            if epoch != self.preview_epoch:
                for points in self.preview.values(): points.clear()
                self.preview_arrival.clear()
                self.recorded.clear()
                self.was_recording = False
                self.preview_epoch, self.preview_origin = epoch, stamp
            t = (stamp-self.preview_origin)/1000000
            recording = row.get('recording', False)
            if recording:
                if self.was_recording and self.recorded:
                    self.recorded[-1][1] = max(self.recorded[-1][1], t)
                else:
                    self.recorded.append([t, t])
            self.was_recording = recording
            bucket = int(t*50)
            for name, values in groups.items():
                points = self.preview[name]
                self.preview_arrival[name] = time.monotonic()
                if points and points[-1][0] == bucket:
                    p = points[-1]
                    p[2] = [min(a,b) for a,b in zip(p[2], values)]
                    p[3] = [max(a,b) for a,b in zip(p[3], values)]
                    p[4] = values
                else:
                    points.append([bucket, t, list(values), list(values), values])

    def chart(self):
        with self.lock:
            now = time.monotonic()
            return dict(epoch=self.preview_epoch, recorded=[p[:] for p in self.recorded],
                        series={k: [[p[1], p[2][:], p[3][:], p[4][:]] for p in v] for k,v in self.preview.items()},
                        stale={k: now-self.preview_arrival.get(k, now-10)>1 for k in self.preview})

    def start(self, port, directory, name, baud, preview_only=False, preserve=False):
        with self.lock:
            promoting = self.state == 'previewing' and not preview_only
            if self.thread and self.thread.is_alive() and not promoting:
                raise ValueError('已有录制正在运行或正在生成 merged.csv')
            if port not in {p.device for p in list_ports.comports()}:
                raise ValueError('串口不存在，请刷新串口列表')
            output = None
            if not preview_only:
                if not name or name in ('.', '..') or '/' in name or '\\' in name:
                    raise ValueError('录制名称应为单个文件夹名称')
                parent = Path(directory).expanduser().resolve()
                if not parent.is_dir():
                    raise ValueError('请选择已存在的保存目录')
                output = parent/name
                if output.exists():
                    raise ValueError('录制目录已存在，请使用新的名称')
            baud = int(baud)
            if baud not in (115200, 1500000, 3000000):
                raise ValueError('请选择支持的波特率')
            if promoting:
                if self.connection != (port, baud):
                    raise ValueError('请先关闭预览，再切换串口或波特率')
                self.output = str(output)
                self.state = 'recording'
                self.report('开始录制；保留当前串口连接，不补写此前预览数据')
                return
            self.connection = (port, baud)
            if not preserve:
                self.logs.clear()
                self.preview_epoch = None
                for points in self.preview.values(): points.clear()
                self.preview_arrival.clear()
            self.output, self.state, self.started = (str(output) if output else ''), ('previewing' if preview_only else 'recording'), time.time()
            args = ['--port', port, '--baud', str(baud), '--diagnostics']
            if output: args += ['--output', str(output)]
            self.thread = threading.Thread(target=self._run, args=(args,), daemon=False)
            self.thread.start()

    def _run(self, args):
        code = 1
        try:
            code = record(args, control=self.control, report=self.report, on_sample=self.sample, output_provider=lambda: self.output)
        except BaseException as exc:
            self.report(f'录制失败：{exc}')
        finally:
            with self.lock:
                was_capture = bool(self.output)
                if was_capture: self.last_output = self.output
                self.state = ('finished' if was_capture else 'idle') if code == 0 else 'error'
                self.report(('录制结束，merged.csv 已生成；继续预览' if was_capture else '已关闭预览') if code == 0 else '请检查日志；已保存的原始CSV可用于重新导出')
                self.was_recording = False
                if code == 0 and was_capture and not self.shutting_down:
                    port, baud = self.connection
                    self.thread = None
                    self.start(port, '', '', baud, preview_only=True, preserve=True)

    def switch_preview(self, port, baud):
        baud = int(baud)
        with self.lock:
            if self.state in ('recording', 'paused', 'stopping'):
                raise ValueError('录制或保存期间不能切换串口设置')
            if port not in {p.device for p in list_ports.comports()}:
                raise ValueError('串口不存在，请刷新设备列表')
            if baud not in (115200, 1500000, 3000000):
                raise ValueError('请选择支持的波特率')
            if self.state == 'previewing' and self.connection == (port, baud):
                return
            worker = self.thread
            if worker and worker.is_alive():
                self.state = 'stopping'
        # Reader needs the lock to close; never join it while holding that lock.
        if worker and worker.is_alive():
            worker.join(timeout=5)
            if worker.is_alive():
                raise ValueError('旧串口尚未关闭，请稍后重试')
        self.start(port, '', '', baud, preview_only=True)

    def transition(self, action):
        with self.lock:
            allowed = {'pause': ('recording', 'paused'), 'resume': ('paused', 'recording')}
            if action == 'stop':
                if self.state not in ('recording', 'paused', 'previewing'):
                    raise ValueError('当前没有可以停止的录制')
                self.state = 'stopping'
                self.report('正在结束录制并生成 merged.csv，请稍候…' if self.output else '正在关闭预览…')
            elif action in allowed:
                old, new = allowed[action]
                if self.state != old:
                    raise ValueError('当前状态不允许此操作')
                self.state = new
                self.report('已暂停写入；串口继续读取' if action == 'pause' else '已继续录制')
            else:
                raise ValueError('未知操作')

    def snapshot(self):
        with self.lock:
            return dict(state=self.state, connection=self.connection, output=self.output or self.last_output, logs=list(self.logs), started=self.started)


def choose_directory():
    if sys.platform != 'darwin':
        raise ValueError('此系统请在网页中手动填写保存目录的绝对路径')
    # No user-provided text is interpolated into AppleScript.
    result = subprocess.run(['osascript', '-e',
        'POSIX path of (choose folder with prompt "选择录制保存目录")'],
        capture_output=True, text=True, timeout=120)
    if result.returncode:
        raise ValueError('已取消目录选择，或系统未允许弹出选择窗口；也可手动填写路径')
    return result.stdout.strip()


def make_handler(recorder, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass

        def respond(self, data, status=200):
            body = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def authorized(self):
            # Prevent unrelated websites from operating the local recorder.
            return secrets.compare_digest(self.headers.get('X-Recorder-Token', ''), token)

        def valid_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.valid_host():
                self.respond({'error': 'Invalid host'}, 403); return
            if self.path == '/':
                html = Path(__file__).with_name('recorder.html').read_text().replace('__TOKEN__', token)
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Frame-Options', 'DENY')
                self.end_headers()
                self.wfile.write(html.encode())
                return
            if not self.valid_host() or not self.authorized():
                self.respond({'error': 'Unauthorized'}, 403); return
            if self.path == '/api/status':
                self.respond(recorder.snapshot())
            elif self.path == '/api/chart':
                self.respond(recorder.chart())
            elif self.path == '/api/config':
                parent = ROOT/'out'
                parent.mkdir(exist_ok=True)
                self.respond(dict(directory=str(parent), name=datetime.now().strftime('run_%Y%m%d_%H%M%S'),
                                  ports=[dict(device=p.device, description=p.description) for p in list_ports.comports()]))
            else:
                self.respond({'error': 'Not found'}, 404)

        def do_POST(self):
            if not self.valid_host() or not self.authorized():
                self.respond({'error': 'Unauthorized'}, 403); return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 8192: raise ValueError('请求过大')
                data = json.loads(self.rfile.read(length) or b'{}')
                if self.path == '/api/preview':
                    recorder.start(data['port'], '', '', data['baud'], preview_only=True)
                    self.respond(recorder.snapshot())
                elif self.path == '/api/connection':
                    recorder.switch_preview(data['port'], data['baud'])
                    self.respond(recorder.snapshot())
                elif self.path == '/api/start':
                    recorder.start(data['port'], data['directory'], data['name'], data['baud'])
                    self.respond(recorder.snapshot())
                elif self.path == '/api/choose':
                    self.respond({'directory': choose_directory()})
                elif self.path in ('/api/pause', '/api/resume', '/api/stop'):
                    recorder.transition(self.path.rsplit('/', 1)[1])
                    self.respond(recorder.snapshot())
                else:
                    self.respond({'error': 'Not found'}, 404)
            except (ValueError, KeyError, OSError, subprocess.TimeoutExpired) as exc:
                self.respond({'error': str(exc)}, 400)
    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=0, help='Local webpage port; default chooses a free port')
    p.add_argument('--no-browser', action='store_true')
    args = p.parse_args()
    recorder = Recorder()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(recorder, secrets.token_hex(32)))
    url = f'http://127.0.0.1:{server.server_port}/'
    print(f'录制控制台：{url}\n请用网页中的“结束录制”保存结果；关闭网页不会自动停止录制。', flush=True)
    if not args.no_browser: webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.shutting_down = True
        if recorder.control() in ('recording', 'paused', 'previewing'): recorder.transition('stop')
        if recorder.thread: recorder.thread.join()
        server.server_close()


if __name__ == '__main__':
    main()
