"""CRSF telemetry from passive CAN reception; never sends CAN requests.

Battery unknown fields use all-FF, which EdgeTX treats as unavailable.
Motor Iq is deliberately not presented as battery current.
"""
import math
import socket
import struct
import threading
import time
from pathlib import Path
from crsf_monitor import crc8


def frame(kind, payload):
    body = bytes([kind]) + payload
    if len(body) > 61:
        raise ValueError('CRSF frame too long')
    return bytes([0xC8, len(body)+1]) + body + bytes([crc8(body)])


def battery(volts):
    if not math.isfinite(volts) or not 0 < volts < 100:
        raise ValueError('Invalid battery voltage')
    return frame(0x08, struct.pack('>H', round(volts*10)) + b'\xff'*6)


def mode_frame(mode):
    return frame(0x21, mode.encode('ascii')[:15] + b'\0')


def temperature(celsius):
    if not math.isfinite(celsius) or not -100 < celsius < 200:
        raise ValueError('Invalid temperature')
    return frame(0x0D, b'\0' + struct.pack('>h', round(celsius*10)))


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.voltages = {}
        self.axes = {}
        self.sent = 0
        self.error = ''
        self.summary = {}
        threading.Thread(target=self.listen, daemon=True).start()

    def listen(self):
        while True:
            try:
                with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
                    # No CAN send calls in this class; existing firmware broadcasts.
                    sock.bind(('can0',))
                    while True:
                        ident, length, data = struct.unpack('=IB3x8s', sock.recv(16))
                        if ident & 0xE0000000 or length != 8:
                            continue
                        node, kind = ident >> 5, ident & 31
                        if node not in range(4):
                            continue
                        now = time.monotonic()
                        with self.lock:
                            if kind == 0x17:
                                volts = struct.unpack_from('<f', data)[0]
                                if math.isfinite(volts) and 0 < volts < 100:
                                    self.voltages[node] = (volts, now)
                            elif kind == 1:
                                self.axes[node] = (struct.unpack_from('<I',data)[0], data[4], now)
            except OSError as e:
                self.error = str(e)
                time.sleep(1)

    def packets(self, radio, enabled, now):
        with self.lock:
            voltages = {n:v for n,(v,at) in self.voltages.items() if now-at < 2}
            fault = any(now-at < .5 and error not in (0,2048)
                        for error, state, at in self.axes.values())
            ready = len(self.axes)==4 and all(now-at < .5 and state==8 and error==0
                                              for error,state,at in self.axes.values())
        mode = ('FAULT' if fault else 'STOP' if radio['held'] else
                'NEUTRAL' if radio['mode'] == 'manual' and radio['gear'] == 0 else
                'DISARMED' if not enabled else 'ARMING' if not ready else radio['mode'].upper())
        packets = [mode_frame(mode)]
        self.summary = dict(mode=mode, vbat=min(voltages.values()) if voltages else None,
                            voltage_nodes=sorted(voltages), sent=self.sent)
        if voltages:
            packets.append(battery(min(voltages.values())))
        try:
            cpu = float(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000
            packets.append(temperature(cpu))
            self.summary['cpu_c'] = cpu
        except (OSError, ValueError):
            pass
        return packets
