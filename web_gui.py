#!/usr/bin/env python3
"""Local-only USB rig GUI: python web_gui.py --config config.json."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import queue
import secrets
import signal
import threading
from urllib.parse import urlparse
import zipfile
from web_rig import Rig


def saved_runs(root):
    result = []
    for path in sorted(root.iterdir(), reverse=True):
        if not path.is_dir() or not (path/'metadata.json').exists():
            continue
        try:
            meta = json.loads((path/'metadata.json').read_text())
            cfg = meta['config']
            result.append({'id': path.name, 'material': cfg.get('material', ''),
                           'angle': cfg.get('geometry', {}).get('needle_angle_deg'),
                           'insertion': cfg.get('engagement_level'), 'trial': cfg.get('trial_id'),
                           'peak': meta.get('peak_abs_sensor_force_N'), 'reason': meta.get('stop_reason'),
                           'status': meta.get('status'), 'outcome': meta.get('outcome'), 'demo': meta.get('demo', False)})
        except (ValueError, KeyError, OSError):
            continue
        if len(result) >= 100:
            break
    return result


def make_handler(rig, token, port):
    web = Path(__file__).parent/'web'
    allowed_origins = {f'http://127.0.0.1:{port}', f'http://localhost:{port}'}
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body, mime='application/json', filename=None):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            if filename:
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/api/state':
                self.send(200, rig.snapshot())
            elif path == '/api/runs':
                self.send(200, saved_runs(rig.root))
            elif path.startswith('/api/download/'):
                ident = path.split('/')[-1]
                root = (rig.root/ident).resolve()
                if root.parent != rig.root.resolve() or not root.is_dir():
                    self.send(404, {'error': 'Run not found'}); return
                archive = io.BytesIO()
                with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as stream:
                    for name in ('samples.csv', 'metadata.json', 'analyzed.csv', 'summary.json'):
                        file = root/name
                        if file.is_file():
                            stream.writestr(name, file.read_bytes())
                self.send(200, archive.getvalue(), 'application/zip', ident+'.zip')
            elif path in ('/', '/app.js', '/style.css'):
                name = 'index.html' if path == '/' else path[1:]
                body = (web/name).read_text()
                if name == 'index.html':
                    body = body.replace('__TOKEN__', token)
                mime = {'index.html': 'text/html; charset=utf-8', 'app.js': 'text/javascript; charset=utf-8',
                        'style.css': 'text/css; charset=utf-8'}[name]
                self.send(200, body, mime)
            else:
                self.send(404, {'error': 'Not found'})

        def do_POST(self):
            if self.headers.get('Origin') not in allowed_origins or self.headers.get('X-Rig-Token') != token:
                self.send(403, {'error': 'Request must come from the local rig page'}); return
            path = urlparse(self.path).path
            if path not in ('/api/tare', '/api/start', '/api/finish', '/api/speed'):
                self.send(404, {'error': 'Unknown action'}); return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 <= size <= 12000:
                    raise ValueError('Request too large')
                values = json.loads(self.rfile.read(size) or b'{}')
                if not isinstance(values, dict):
                    raise ValueError('Expected object')
                rig.enqueue(path.split('/')[-1], values)
                self.send(202, {'accepted': True})
            except (ValueError, queue.Full) as exc:
                self.send(400, {'error': str(exc) or 'Command queue full'})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config.json')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--demo', action='store_true', help='Synthetic force/motion; no USB access')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        raise ValueError('Port must be 1024–65535')
    rig = Rig(args.config, demo=args.demo)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(rig, secrets.token_urlsafe(32), args.port))
    rig.launch()
    def shutdown_signal(*_):
        # Worker immediately receives quit; server.shutdown cannot run in serve thread.
        rig.quit.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown_signal)
    print(f"{'DEMO — ' if args.demo else ''}Needle test GUI: http://127.0.0.1:{args.port}", flush=True)
    print('Close Arduino Serial Monitor and CLI loggers. USB connects only when you click Tare.', flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rig.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
