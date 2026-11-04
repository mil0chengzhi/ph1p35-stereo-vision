"""Local phone UI -> explicitly selected Bluetooth SPP port. No board programming."""
from __future__ import annotations
import argparse
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
import time
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent
STATE_LOCK = threading.RLock()
SERIAL = None
RX = deque(maxlen=128)
RX_ID = 0
ERROR = None


def available_ports():
    from serial.tools import list_ports
    # Only Bluetooth ports; do not offer board USB/JTAG/debug ports.
    return [dict(device=p.device, description=p.description)
            for p in list_ports.comports()
            if any(k in (p.hwid + ' ' + p.description).upper()
                   for k in ('BTHENUM', 'BLUETOOTH', '蓝牙'))]


def disconnect():
    global SERIAL
    with STATE_LOCK:
        old, SERIAL = SERIAL, None
    if old:
        old.close()


def read_serial(link):
    global RX_ID, ERROR, SERIAL
    try:
        while SERIAL is link:
            data = link.read(max(1, min(link.in_waiting, 256)))
            if data:
                with STATE_LOCK:
                    RX_ID += 1
                    RX.append(dict(id=RX_ID, bytes=list(data)))
    except Exception as exc:
        with STATE_LOCK:
            if SERIAL is link:
                ERROR = str(exc)
                SERIAL = None
        try:
            link.close()
        except Exception:
            pass


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT.parents[1] / 'docs' / 'remote'), **kwargs)

    def json_reply(self, data, code=200):
        raw = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        parsed = urlparse(self.path)
        try:
            if parsed.path == '/api/ports':
                return self.json_reply(dict(ports=available_ports()))
            if parsed.path == '/api/status':
                since = int(parse_qs(parsed.query).get('since', ['0'])[0])
                with STATE_LOCK:
                    return self.json_reply(dict(connected=SERIAL is not None,
                        last_rx_id=RX_ID, rx=[r for r in RX if r['id'] > since], link_error=ERROR))
            if parsed.path.startswith('/api/'):
                return self.json_reply(dict(error='Unknown endpoint'), 404)
            return super().do_GET()
        except Exception as exc:
            self.json_reply(dict(error=str(exc)), 400)

    def do_POST(self):
        global SERIAL, ERROR
        try:
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                raise ValueError('Only same-origin requests are accepted')
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                raise ValueError('Expected application/json')
            size = int(self.headers.get('Content-Length', 0))
            if not 0 < size <= 1024:
                raise ValueError('Invalid request length')
            body = json.loads(self.rfile.read(size))
            if self.path == '/api/connect':
                import serial
                port = body.get('port')
                baud = body.get('baud')
                if port not in {p['device'] for p in available_ports()}:
                    raise ValueError('Select a listed Bluetooth SPP port, not a board USB port')
                if baud not in (9600, 38400, 115200):
                    raise ValueError('Unsupported baud rate')
                disconnect()
                link = serial.Serial(port, baudrate=baud, timeout=0.15, write_timeout=1)
                with STATE_LOCK:
                    SERIAL = link
                    ERROR = None
                    RX.clear()
                threading.Thread(target=read_serial, args=(link,), daemon=True).start()
                return self.json_reply(dict(connected=True, last_rx_id=RX_ID))
            if self.path == '/api/disconnect':
                disconnect()
                return self.json_reply(dict(connected=False))
            if self.path == '/api/distance':
                mm = body.get('mm')
                if type(mm) is not int or not 500 <= mm <= 750:
                    raise ValueError('Distance must be an integer from 500 to 750 millimeters')
                seq = body.get('seq')
                if type(seq) is not int or not 0 <= seq <= 255:
                    raise ValueError('Sequence must be an integer from 0 to 255')
                payload = distance_packet(mm, seq)
                with STATE_LOCK:
                    if SERIAL is None:
                        raise ValueError('Bluetooth serial port is not connected')
                    count = SERIAL.write(payload)
                if count != len(payload):
                    raise IOError('Incomplete serial write')
                # A host write is deliberately not a board/OSD acknowledgement.
                return self.json_reply(dict(written=True, frame_hex=payload.hex(), board_ack=False))
            return self.json_reply(dict(error='Unknown endpoint'), 404)
        except Exception as exc:
            return self.json_reply(dict(error=str(exc)), 400)


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (7 if value & 128 else 0)) & 255
    return value


def distance_packet(mm, seq):
    frame = bytes((0xA5, 0x5A, seq, 0x20, mm & 255, mm >> 8, 0))
    return frame + bytes((crc8(frame),))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8796)
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'Computer: http://127.0.0.1:{args.port}/', flush=True)
    for address in sorted({a[4][0] for a in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}):
        if not address.startswith('127.'):
            print(f'Phone, same network: http://{address}:{args.port}/', flush=True)
    print('No serial port is opened until you select one and press Connect.', flush=True)
    try:
        server.serve_forever()
    finally:
        disconnect()
        server.server_close()


if __name__ == '__main__':
    main()
