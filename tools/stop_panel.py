#!/usr/bin/env python3
"""Independent latched browser stop. No CAN and no motor-start operation.

Only the new ros2_control plugin consumes this flag; old robot_base must be stopped.
Browser disconnect/closure has no effect. Server startup latches STOP.
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

class StopState:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
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
            self.enabled = True
            self.reason = 'СТОП разблокирован. Само по себе это не включает колёса.'
            self.write()

    def status(self):
        with self.lock:
            return {'ready': self.enabled, 'reason': self.reason}

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--address', default='192.168.67.149')
    p.add_argument('--port', type=int, default=8081)
    p.add_argument('--flag', default='/run/mrbobus-stop/enabled')
    p.add_argument('--page', required=True)
    args = p.parse_args()
    state = StopState(args.flag)
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
                state.ready(); self.reply(200, state.status())
            else: self.reply(404, {})
    class Server(ThreadingHTTPServer):
        daemon_threads = True
        def get_request(self):
            sock, address = super().get_request()
            sock.settimeout(2)
            return sock, address
    server = Server((args.address, args.port), Handler)
    try: server.serve_forever(poll_interval=.1)
    finally: state.stop('STOP: сервер выключен'); server.server_close()

if __name__ == '__main__': main()
