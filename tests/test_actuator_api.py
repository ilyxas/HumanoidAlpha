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
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
from control_console import Controller
from actuator_runtime import ActuatorAPI, validate


class VectorTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller()

    def test_lengths_and_types(self):
        for v in ([], [0]*32, [0]*34, '0', [[0]]*33, [True]*33, ['0']*33):
            with self.assertRaises(ValueError): validate(v, self.c.m)

    def test_nonfinite_and_limits(self):
        for value in (float('nan'), float('inf'), -float('inf'), 10**1000):
            v = [0]*33; v[7] = value
            with self.assertRaises((ValueError, OverflowError)): validate(v, self.c.m)
        for i in range(33):
            for edge in self.c.m.actuator_ctrlrange[i]:
                v = [0.0]*33; v[i] = float(edge)
                validate(v, self.c.m)
            for value in (self.c.m.actuator_ctrlrange[i,0]-.01, self.c.m.actuator_ctrlrange[i,1]+.01):
                v = [0.0]*33; v[i] = float(value)
                with self.assertRaises(ValueError): validate(v, self.c.m)

    def test_atomic_order_persistence_zero_and_motion(self):
        async def check():
            with tempfile.TemporaryDirectory() as tmp:
                api = ActuatorAPI(self.c, asyncio.Queue(), object(), Path(tmp)/'s', Path(tmp)/'log')
                api.log = open(api.log_path, 'a', buffering=1)
                try:
                    vector = [float(i+1)/100 for i in range(33)]
                    done = asyncio.get_running_loop().create_future()
                    api.apply((vector, done))
                    self.assertEqual(await done, 'OK')
                    np.testing.assert_array_equal(self.c.d.ctrl, vector)
                    self.assertEqual([a['id'] for a in self.c.inventory['actuators']], list(range(33)))
                    self.assertEqual([a['name'] for a in self.c.inventory['actuators']], [self.c.m.actuator(i).name for i in range(33)])
                    bad = vector.copy(); bad[32] = 1e9
                    done = asyncio.get_running_loop().create_future(); api.apply((bad, done))
                    self.assertTrue((await done).startswith('ERROR'))
                    np.testing.assert_array_equal(self.c.d.ctrl, vector)
                    self.c.mode, self.c.paused = 'dynamic', False
                    before = self.c.snapshot()['pose']
                    for _ in range(100): self.c.tick()
                    self.assertGreater(self.c.d.time, 0)
                    np.testing.assert_array_equal(self.c.d.ctrl, vector)
                    self.assertNotEqual(before, self.c.snapshot()['pose'])
                    done = asyncio.get_running_loop().create_future(); api.apply(([0.0]*33, done))
                    self.assertEqual(await done, 'OK')
                    np.testing.assert_array_equal(self.c.d.ctrl, np.zeros(33))
                    self.assertFalse(self.c.paused)
                    records = [json.loads(x) for x in api.log_path.read_text().splitlines()]
                    self.assertEqual([x['result'] for x in records], ['accepted','rejected','accepted'])
                    self.assertEqual(records[0]['values'], vector)
                    self.assertGreater(records[-1]['sim_time'], 0)
                    self.assertTrue(records[0]['wall_time'])
                finally: api.log.close()
        asyncio.run(check())

    def test_cli_invalid_input_is_opaque(self):
        for args in (['act','1'], ['act']+['nan']*33, ['act']+['inf']*33, ['zero','1']):
            r = subprocess.run([sys.executable, str(ROOT/'humanoid'), *args], capture_output=True, text=True)
            self.assertEqual(r.returncode, 2)
            self.assertEqual(r.stdout, '')
            self.assertEqual(r.stderr, 'ERROR use act with 33 finite numbers, or zero\n')


class RuntimeTests(unittest.TestCase):
    """Real separate CLI processes, local IPC and developer WS; requires loopback permission."""
    def test_persistent_runtime_and_console_observer(self):
        with tempfile.TemporaryDirectory(prefix='ha-') as tmp:
            sockpath = str(Path(tmp)/'act.sock')
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
            env = dict(os.environ, HUMANOID_ACTUATOR_SOCKET=sockpath, PYTHONDONTWRITEBYTECODE='1')
            log = Path(tmp)/'commands.jsonl'
            proc = subprocess.Popen([sys.executable, str(ROOT/'physics/control_console.py'), '--port', str(port), '--actuator-api', '--actuator-log', str(log)], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                for _ in range(100):
                    if Path(sockpath).exists(): break
                    if proc.poll() is not None: self.fail(proc.communicate()[1].decode())
                    time.sleep(.05)
                self.assertTrue(Path(sockpath).exists())
                self.assertEqual(Path(sockpath).stat().st_mode & 0o777, 0o600)
                async def check():
                    import websockets
                    async with websockets.connect(f'ws://127.0.0.1:{port}') as ws:
                        inv = json.loads(await ws.recv())
                        self.assertEqual(inv['type'], 'inventory'); self.assertFalse(inv['can_control'])
                        async def pose():
                            while True:
                                msg = json.loads(await asyncio.wait_for(ws.recv(), 5))
                                if msg['type']=='pose': return msg
                        async def cli(args):
                            p = await asyncio.create_subprocess_exec(sys.executable, str(ROOT/'humanoid'), *args, env=env, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                            stdout, stderr = await p.communicate()
                            self.assertEqual(p.returncode,0,stderr.decode()); self.assertEqual(stdout,b'OK\n'); self.assertEqual(stderr,b'')
                        # Transport-side validation is independent of the CLI parser.
                        for values in ([0]*32, [0]*34, [float('nan')]*33, [float('inf')]*33, [1e9]*33):
                            reader, writer = await asyncio.open_unix_connection(sockpath)
                            writer.write((json.dumps({'values':values})+'\n').encode())
                            await writer.drain()
                            self.assertTrue((await reader.readline()).startswith(b'ERROR'))
                            writer.close(); await writer.wait_closed()
                        v = [(i+1)/100 for i in range(33)]
                        await cli(['act', *map(str,v)])
                        while (a:=await pose())['state']['ctrl'] != v: pass
                        await asyncio.sleep(.1)
                        b = await pose()
                        self.assertGreater(b['sim_time'],a['sim_time']); self.assertEqual(b['state']['ctrl'],v)
                        self.assertNotEqual(a['pose'], b['pose'])
                        await ws.send(json.dumps({'op':'reset'}))
                        while True:
                            msg=json.loads(await ws.recv())
                            if msg['type']=='error': break
                        v2 = [-x for x in v]
                        await cli(['act', *map(str,v2)])
                        while (b:=await pose())['state']['ctrl']!=v2: pass
                        self.assertGreater(b['sim_time'],a['sim_time'])
                        await cli(['zero'])
                        while (b:=await pose())['state']['ctrl']!=[0.0]*33: pass
                        self.assertFalse(b['state']['paused'])
                        self.assertIsNone(proc.poll())
                    # Browser observer disconnect must not stop API physics.
                    await asyncio.sleep(.1)
                    async with websockets.connect(f'ws://127.0.0.1:{port}') as ws2:
                        await ws2.recv()
                        later=json.loads(await ws2.recv())
                        self.assertGreater(later['sim_time'], b['sim_time'])
                        self.assertFalse(later['state']['paused'])
                asyncio.run(asyncio.wait_for(check(),15))
                records = [json.loads(x) for x in log.read_text().splitlines()]
                self.assertEqual([r['result'] for r in records], ['rejected']*5+['accepted']*3)
            finally:
                proc.terminate()
                try: proc.communicate(timeout=5)
                except subprocess.TimeoutExpired: proc.kill(); proc.communicate()

if __name__ == '__main__': unittest.main()
