"""HTTPS LAN panel. HTTP threads cannot touch CAN or the state machine."""
import hashlib
import hmac
import http.cookies
import ipaddress
import json
import secrets
import ssl
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

class Panel:
    def __init__(self, address, port, password_path, cert, key, web_root, dispatch, snapshot, require_password=True):
        self.require_password = require_password
        self.password = Path(password_path).read_text().strip() if require_password else ''
        if require_password and len(self.password) < 20:
            raise ValueError('Panel password must have at least 20 characters')
        self.sessions = {}
        self.attempts = {}
        self.lock = threading.Lock()
        self.origin = f'https://{address}:{port}'
        self.root = Path(web_root)
        self.dispatch, self.snapshot = dispatch, snapshot
        panel = self
        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'  # Reuse TLS connections for 20 Hz commands.
            def log_message(self, *_):
                pass  # Never log cookies or login payloads.

            def reply(self, code, body, content='application/json', cookie=None):
                if not isinstance(body, bytes):
                    body = json.dumps(body).encode()
                self.send_response(code)
                self.send_header('Content-Type', content)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Content-Security-Policy', "default-src 'self'; frame-ancestors 'none'; object-src 'none'")
                if cookie:
                    self.send_header('Set-Cookie', cookie)
                self.end_headers()
                self.wfile.write(body)

            def session(self):
                if not panel.require_password:
                    return 'lan'  # Per-tab UUID and per-arm lease still enforce one operator.
                try:
                    cookies = http.cookies.SimpleCookie(self.headers.get('Cookie',''))
                    sid = cookies['robot_session'].value
                except (KeyError, http.cookies.CookieError):
                    return None
                with panel.lock:
                    if panel.sessions.get(sid, 0) > time.monotonic():
                        return sid
                return None

            def do_GET(self):
                if self.path in ('/', '/app.js', '/style.css'):
                    filename = {'/': 'index.html', '/app.js': 'app.js', '/style.css':'style.css'}[self.path]
                    content = {'/':'text/html; charset=utf-8','/app.js':'text/javascript','/style.css':'text/css'}[self.path]
                    self.reply(200, (panel.root/filename).read_bytes(), content)
                elif self.path == '/api/status' and self.session():
                    self.reply(200, panel.snapshot())
                else:
                    self.reply(401, {'error':'Login required'})

            def do_POST(self):
                if self.headers.get('Origin') != panel.origin:
                    self.reply(403, {'error':'Wrong origin'})
                    return
                if self.headers.get('Content-Type','').split(';')[0] != 'application/json':
                    self.reply(415, {'error':'JSON required'})
                    return
                try:
                    size = int(self.headers.get('Content-Length','0'))
                    if not 0 < size <= 2048:
                        raise ValueError()
                    data = json.loads(self.rfile.read(size))
                    if not isinstance(data, dict):
                        raise ValueError()
                except (ValueError, OSError):
                    self.reply(400, {'error':'Invalid request'})
                    return
                if self.path == '/api/login':
                    if not panel.require_password:
                        self.reply(200, {'ok':True})
                        return
                    now = time.monotonic()
                    ip = self.client_address[0]
                    with panel.lock:
                        recent = [t for t in panel.attempts.get(ip,[]) if now-t < 60]
                        panel.attempts[ip] = recent+[now]
                        if len(recent) >= 10:
                            self.reply(429, {'error':'Wait one minute'})
                            return
                        supplied = str(data.get('password',''))
                        if not hmac.compare_digest(hashlib.sha256(supplied.encode()).digest(),
                                                   hashlib.sha256(panel.password.encode()).digest()):
                            self.reply(401, {'error':'Invalid password'})
                            return
                        panel.sessions = {s:t for s,t in panel.sessions.items() if t > now}
                        if len(panel.sessions) >= 32:
                            self.reply(429, {'error':'Too many sessions'})
                            return
                        sid = secrets.token_urlsafe(32)
                        panel.sessions[sid] = now+8*3600
                    self.reply(200, {'ok':True}, cookie=f'robot_session={sid}; Secure; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800')
                    return
                sid = self.session()
                if not sid:
                    self.reply(401, {'error':'Login required'})
                    return
                tab = data.get('tab','')
                if not isinstance(tab,str) or not 8 <= len(tab) <= 80:
                    self.reply(400, {'error':'Tab identifier required'})
                    return
                op = self.path.removeprefix('/api/')
                if op not in ('arm','command','stop','reset'):
                    self.reply(404, {'error':'Unknown operation'})
                    return
                answer = panel.dispatch(op, sid+':'+tab, data)
                self.reply(200 if answer.get('ok') else 409, answer)

        if not ipaddress.ip_address(address).is_private:
            raise ValueError('Bind only a private LAN address')
        class Server(ThreadingHTTPServer):
            daemon_threads = True
            request_queue_size = 16
            def get_request(self):
                sock, addr = super().get_request()
                sock.settimeout(2)
                return sock, addr
        self.server = Server((address,port), Handler)
        tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls.minimum_version = ssl.TLSVersion.TLSv1_2
        tls.load_cert_chain(cert, key)
        self.server.socket = tls.wrap_socket(self.server.socket, server_side=True)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
