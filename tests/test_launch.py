"""Isolated launch/stop checks; safe to run alongside a live experiment."""
import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from launch import stop_managed


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class LaunchTests(unittest.TestCase):
    def test_both_modes_redirect_ownership_and_force_stop(self):
        for mode in ('debug', 'experiment'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory(prefix='ha-launch-') as directory:
                root = Path(directory)
                for name in ('physics', 'viewer', 'assets'):
                    (root / name).symlink_to(ROOT / name, target_is_directory=True)
                (root / 'reports').mkdir()
                http_port, ws_port = free_port(), free_port()
                env = dict(os.environ, HUMANOID_ACTUATOR_SOCKET=str(root / 'act.sock'))
                # Override only the isolated launcher's root. The user's runtime
                # uses other ports, lock file, actuator socket and log file.
                code = ('import sys; from pathlib import Path; from argparse import Namespace; '
                        f'sys.path.insert(0,{str(ROOT / "scripts")!r}); import launch; '
                        f'launch.ROOT=Path({str(root)!r}); '
                        f'launch.run(Namespace(mode={mode!r}, http_port={http_port}, '
                        f'ws_port={ws_port}, no_open=True))')
                process = subprocess.Popen([sys.executable, '-c', code], env=env,
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    for _ in range(150):
                        if process.poll() is not None:
                            self.fail(process.communicate()[1].decode())
                        try:
                            with urlopen(f'http://127.0.0.1:{http_port}/', timeout=.2) as response:
                                target = 'console.html' if mode == 'debug' else 'observation.html'
                                self.assertIn(target, response.url)
                                self.assertIn(f'wsPort={ws_port}', response.url)
                                self.assertEqual(response.headers['Cache-Control'], 'no-cache')
                                break
                        except OSError:
                            time.sleep(.05)
                    else:
                        self.fail('HTTP did not start')
                    async def inspect():
                        import websockets
                        for _ in range(100):
                            try:
                                async with websockets.connect(f'ws://127.0.0.1:{ws_port}') as ws:
                                    inventory = json.loads(await ws.recv())
                                    self.assertEqual(inventory['can_control'], mode == 'debug')
                                    pose = json.loads(await ws.recv())
                                    self.assertEqual(pose['state']['mode'], 'kinematic' if mode == 'debug' else 'dynamic')
                                    return
                            except OSError:
                                await asyncio.sleep(.05)
                        self.fail('WebSocket did not start')
                    # Let the supervisor's readiness connection close first.
                    time.sleep(.3)
                    asyncio.run(inspect())
                    duplicate = subprocess.run([sys.executable, '-c', code], env=env,
                                               capture_output=True, timeout=5)
                    self.assertNotEqual(duplicate.returncode, 0)
                    self.assertIsNone(process.poll())
                    self.assertTrue(stop_managed(root))
                    process.communicate(timeout=5)
                    for port in (http_port, ws_port):
                        with socket.socket() as probe:
                            self.assertNotEqual(probe.connect_ex(('127.0.0.1', port)), 0)
                finally:
                    if process.poll() is None:
                        process.terminate()
                    process.communicate(timeout=5)


if __name__ == '__main__':
    unittest.main()
