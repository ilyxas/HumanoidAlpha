"""One supervisor for the debug console and the experiment runtime (Mac/Linux)."""
import argparse
import fcntl
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import signal
import socket
import socketserver
import stat
import subprocess
import sys
import threading
import time
import webbrowser

ROOT = Path(__file__).resolve().parents[1]


def control_path(root=None):
    root = ROOT if root is None else root
    key = hashlib.sha256(str(root.resolve()).encode()).hexdigest()[:16]
    return Path(f'/tmp/humanoid-launcher-{os.getuid()}-{key}.sock')


def stop_managed(root=None):
    """Return False only when no managed launcher is listening for this root."""
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(10)
        try:
            client.connect(str(control_path(root)))
        except (FileNotFoundError, ConnectionRefusedError):
            return False
        client.sendall(b'STOP_FORCE\n')
        reply = client.recv(128)
        if reply != b'STOPPED\n':
            raise RuntimeError('Could not confirm project shutdown')
    return True


def run(args):
    from websockets.sync.client import connect
    os.chdir(ROOT)
    state = ROOT / '.runtime'
    state.mkdir(mode=0o700, exist_ok=True)
    lock = (state / 'launcher.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('Project already running. Use ./START.command stop first.')
    stop = threading.Event()
    stopped = threading.Event()
    force = threading.Event()
    process = None
    http = None
    control = None
    http_started = False
    control_started = False
    path = control_path()
    page = 'console.html' if args.mode == 'debug' else 'observation.html'  # experiment, harness, assisted
    route = f'/viewer/{page}?wsPort={args.ws_port}'

    class Handler(SimpleHTTPRequestHandler):
        def do_HEAD(self):
            if self.path.split('?')[0] == '/':
                self.send_response(302)
                self.send_header('Location', route)
                self.end_headers()
            else:
                super().do_HEAD()

        def do_GET(self):
            if self.path.split('?')[0] == '/':
                self.send_response(302)
                self.send_header('Location', route)
                self.end_headers()
            else:
                super().do_GET()

        def end_headers(self):
            # Reloads must pick up the current UI after a project update.
            self.send_header('Cache-Control', 'no-cache')
            super().end_headers()

        def log_message(self, *_):
            pass

    class StopHandler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(2)
            try:
                if self.rfile.readline(32) != b'STOP_FORCE\n':
                    return
                force.set(); stop.set()
                if stopped.wait(8):
                    self.wfile.write(b'STOPPED\n')
            except (OSError, TimeoutError):
                pass

    class ControlServer(socketserver.ThreadingUnixStreamServer):
        daemon_threads = True

    def request_stop(*_):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        # Check before spawning anything; never connect to or kill an unrelated
        # service that happens to occupy a required port.
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(('127.0.0.1', args.ws_port))
            except OSError as error:
                raise RuntimeError(f'Port {args.ws_port} is busy. Stop the previous project first.') from error
        try:
            http = ThreadingHTTPServer(('127.0.0.1', args.http_port), Handler)
        except OSError as error:
            raise RuntimeError(f'Port {args.http_port} is busy. Stop the previous project first.') from error
        if path.exists():
            info = path.lstat()
            if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
                raise RuntimeError('Unexpected launcher socket; refusing to replace it.')
            path.unlink()  # This root's exclusive lock is held.
        control = ControlServer(str(path), StopHandler)
        os.chmod(path, 0o600)
        threading.Thread(target=control.serve_forever, daemon=True).start()
        control_started = True
        threading.Thread(target=http.serve_forever, daemon=True).start()
        http_started = True
        (ROOT / 'reports').mkdir(exist_ok=True)
        subprocess.run([sys.executable, str(ROOT / 'physics/control_console.py'),
                        '--inventory', str(ROOT / 'reports/control_inventory.json')],
                       cwd=ROOT, check=True)
        command = [sys.executable, str(ROOT / 'physics/control_console.py'), '--port', str(args.ws_port)]
        if args.mode in ('experiment', 'harness', 'assisted'):
            command.extend(['--actuator-api', '--actuator-log', str(ROOT / 'reports/actuator_commands.jsonl')])
        if args.mode == 'assisted':
            command.append('--assisted-stand')
        if args.mode == 'harness':
            command.append('--harness')
        process = subprocess.Popen(command, cwd=ROOT)
        deadline = time.monotonic() + 15
        while not stop.is_set():
            if process.poll() is not None:
                raise RuntimeError('Physics failed to start; see the error above.')
            try:
                with connect(f'ws://127.0.0.1:{args.ws_port}', open_timeout=.3) as connection:
                    connection.recv(timeout=1)
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise RuntimeError('Physics startup timed out.')
                stop.wait(.05)
        url = f'http://127.0.0.1:{args.http_port}/'
        if not stop.is_set():
            print(f'\n{args.mode.upper()} — {url}\nCtrl+C to stop.\n', flush=True)
            if not args.no_open:
                webbrowser.open(url)
        while not stop.wait(.1):
            if process.poll() is not None:
                raise RuntimeError('Physics stopped; closing the viewer server.')
    finally:
        if process is not None and process.poll() is None:
            process.kill() if force.is_set() else process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        if http is not None:
            # shutdown requires a running serve_forever thread.
            if http_started:
                http.shutdown()
            http.server_close()
        stopped.set()
        if control is not None:
            if control_started:
                control.shutdown()
            control.server_close()
            path.unlink(missing_ok=True)
        lock.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['debug', 'experiment', 'harness', 'assisted'])
    parser.add_argument('--no-open', action='store_true', help='Print the URL without opening a browser')
    parser.add_argument('--http-port', type=int, default=8788)
    parser.add_argument('--ws-port', type=int, default=8766)
    try:
        run(parser.parse_args())
    except (RuntimeError, OSError) as error:
        sys.exit(str(error))
