#!/usr/bin/env python3
"""Independent latched browser stop. No CAN and no motor-start operation.

Only the new ros2_control plugin consumes this flag; old robot_base must be stopped.
Browser disconnect/closure has no effect. Server startup latches STOP.
"""
import argparse
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

class StopState:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.RLock()
        self.radio = None
        self.enabled = False
        self.reason = 'STOP: тесты запрещены'
        self.write()

    def write(self):
        temporary = self.path.with_suffix('.new')
        temporary.write_text('1' if self.enabled else '0')
        temporary.replace(self.path)

    def stop(self, reason='STOP нажат'):
        with self.lock:
            self.enabled = False
            self.reason = reason
            self.write()

    def ready(self):
        with self.lock:
            if self.radio and not self.radio.may_arm(time.monotonic()):
                raise ValueError('Пульт запрещает включение: STOP, нейтраль, газ или потеря связи')
            self.enabled = True
            self.reason = 'СТОП разблокирован. Само по себе это не включает колёса.'
            self.write()

    def status(self):
        with self.lock:
            return {'ready': self.enabled, 'reason': self.reason}

def radio_worker(state, port_name):
    import serial
    from crsf_monitor import Parser, channels
    from crsf_telemetry import Telemetry
    telemetry = Telemetry()
    path = state.path.parent/'radio.json'
    previous_stop = 0
    while True:
        try:
            with serial.Serial(port_name, 420000, timeout=.01, write_timeout=.02, exclusive=True) as port:
                parser = Parser()
                last_write = 0.
                last_telemetry = 0.
                while True:
                    data = port.read(port.in_waiting or 1)
                    now = time.monotonic()
                    with state.lock:
                        for kind, payload in parser.feed(data):
                            if kind == 0x14 and len(payload) == 10: state.radio.link(payload[2], now)
                            elif kind == 0x16 and len(payload) == 22: state.radio.update(channels(payload), now)
                        snapshot = state.radio.status(now)
                        if snapshot['stop_token'] != previous_stop:
                            previous_stop = snapshot['stop_token']
                            state.stop(snapshot['reason'])
                        if now-last_telemetry >= 1.:
                            for packet in telemetry.packets(snapshot, state.enabled, now):
                                if port.write(packet) != len(packet):
                                    raise OSError('Incomplete CRSF telemetry write')
                                telemetry.sent += 1
                            last_telemetry = now
                        snapshot['telemetry'] = telemetry.summary
                        if now-last_write > .04:
                            temp = path.with_suffix('.new')
                            temp.write_text(json.dumps(snapshot))
                            temp.replace(path)
                            last_write = now
        except Exception:
            # A dead reader must never leave MANUAL enabled. The console also
            # requires a fresh status file, independent of channel freshness.
            state.stop('STOP: ошибка UART ELRS')
            time.sleep(.2)

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--address', default='192.168.67.149')
    p.add_argument('--port', type=int, default=8081)
    p.add_argument('--flag', default='/run/mrbobus-stop/enabled')
    p.add_argument('--page', required=True)
    p.add_argument('--radio-port')
    args = p.parse_args()
    state = StopState(args.flag)
    if args.radio_port:
        from radio_policy import RadioPolicy
        state.radio = RadioPolicy()
        threading.Thread(target=radio_worker, args=(state,args.radio_port), daemon=True).start()
    page = Path(args.page).read_bytes()
    origin = f'http://{args.address}:{args.port}'

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def reply(self, code, value, content='application/json'):
            data = value if isinstance(value, bytes) else json.dumps(value).encode()
            self.send_response(code)
            self.send_header('Content-Type', content)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            self.wfile.write(data)
        def do_GET(self):
            if self.path == '/': self.reply(200, page, 'text/html; charset=utf-8')
            elif self.path == '/status': self.reply(200, state.status())
            else: self.reply(404, {})
        def do_POST(self):
            if self.headers.get('Origin') != origin:
                self.reply(403, {'error':'origin'}); return
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 512: raise ValueError()
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict): raise ValueError()
            except (ValueError, OSError):
                self.reply(400, {}); return
            if self.path == '/stop':
                state.stop(); self.reply(200, state.status())
            elif self.path == '/ready':
                try:
                    state.ready(); self.reply(200, state.status())
                except ValueError as e: self.reply(409, {'error':str(e)})
            else: self.reply(404, {})
    class Server(ThreadingHTTPServer):
        daemon_threads = True
        def server_bind(self):
            # Radio/STOP must start before DHCP assigns the web panel's IP.
            # Linux FREEBIND retains the specific address and Origin policy;
            # it does not expose the panel on every network interface.
            if sys.platform == 'linux':
                self.socket.setsockopt(socket.SOL_IP, getattr(socket, 'IP_FREEBIND', 15), 1)
            super().server_bind()
        def get_request(self):
            sock, address = super().get_request()
            sock.settimeout(2)
            return sock, address
    server = Server((args.address, args.port), Handler)
    try: server.serve_forever(poll_interval=.1)
    finally: state.stop('STOP: сервер выключен'); server.server_close()

if __name__ == '__main__': main()
