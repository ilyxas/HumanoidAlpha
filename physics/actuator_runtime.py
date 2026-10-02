"""Local opaque actuator transport. Only apply(), called by the physics loop, writes ctrl."""
import asyncio
import json
import math
import os
import socket
from datetime import datetime, timezone
from pathlib import Path


def default_socket():
    return Path(os.environ.get('HUMANOID_ACTUATOR_SOCKET', f'/tmp/humanoid-actuator-{os.getuid()}.sock'))


def validate(values, model):
    if not isinstance(values, list) or len(values) != 33:
        raise ValueError('expected exactly 33 numbers')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('expected finite numbers')
    if any(model.actuator_ctrllimited[i] and not model.actuator_ctrlrange[i, 0] <= v <= model.actuator_ctrlrange[i, 1]
           for i, v in enumerate(values)):
        raise ValueError('value outside control limits')
    return values


def log_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, list):
        return [log_value(v) for v in value]
    if isinstance(value, dict):
        return {k: log_value(v) for k, v in value.items()}
    return value


class ActuatorAPI:
    def __init__(self, controller, queue, owner, path, log_path):
        self.c, self.queue, self.owner = controller, queue, owner
        self.path, self.log_path = Path(path), Path(log_path)
        self.server = self.log = self.inode = None

    async def __aenter__(self):
        if self.c.m.nu != 33:
            raise ValueError('Actuator API requires exactly 33 channels')
        validate([0.0] * 33, self.c.m)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open('a', buffering=1)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            # Never unlink an existing socket: it could belong to another runtime.
            sock.bind(str(self.path))
            self.inode = self.path.stat().st_ino
            self.path.chmod(0o600)
            sock.listen(32)
            sock.setblocking(False)
            self.server = await asyncio.start_unix_server(self.handle, sock=sock, limit=16384)
        except BaseException:
            sock.close()
            await self.__aexit__(None, None, None)
            raise
        return self

    async def __aexit__(self, *args):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        if self.inode and self.path.exists() and self.path.stat().st_ino == self.inode:
            self.path.unlink()
        if self.log:
            self.log.close()

    def record(self, values, accepted):
        self.log.write(json.dumps(dict(wall_time=datetime.now(timezone.utc).isoformat(),
            sim_time=float(self.c.d.time), values=log_value(values),
            result='accepted' if accepted else 'rejected'), allow_nan=False) + '\n')

    def apply(self, request):
        # request is either (values, done) for RAW act, or ('assist', op, done)
        if isinstance(request, tuple) and len(request) == 3 and request[0] == 'assist':
            _, op, done = request
            try:
                if not getattr(self.c, '_assisted_capability', False):
                    raise ValueError('assisted stand not available')
                if op == 'on':
                    self.c.set_assisted(True)
                    reply = 'OK'
                elif op == 'off':
                    self.c.set_assisted(False)
                    reply = 'OK'
                elif op == 'reset':
                    self.c.assist_reset()
                    reply = 'OK'
                elif op == 'status':
                    import json as _json
                    reply = 'OK ' + _json.dumps(self.c.assist_status(), allow_nan=False)
                else:
                    raise ValueError('unknown assist op')
            except (ValueError, TypeError) as e:
                reply = f'ERROR {e}'
            if not done.done():
                done.set_result(reply)
            return

        values, done = request
        try:
            validate(values, self.c.m)
        except (ValueError, OverflowError):
            self.record(values, False)
            reply = 'ERROR invalid actuator vector'
        else:
            self.record(values, True)
            if getattr(self.c, 'assisted', False) and getattr(self.c, 'assist', None) is not None:
                # Store feedforward command; tick applies assist. Also set ctrl once
                # so the first frame is not zero if tick has not run yet.
                self.c.u_cmd[:] = values
                self.c.d.ctrl[:] = self.c.assist.compute(self.c.m, self.c.d, self.c.u_cmd)
            else:
                self.c.d.ctrl[:] = values
                if hasattr(self.c, 'u_cmd'):
                    self.c.u_cmd[:] = values
            self.c.revision += 1
            reply = 'OK'
        if not done.done():
            done.set_result(reply)

    async def handle(self, reader, writer):
        values = None
        try:
            raw = await asyncio.wait_for(reader.readline(), 5)
            message = json.loads(raw)
            if not isinstance(message, dict):
                raise ValueError('invalid request')
            keys = set(message)
            done = asyncio.get_running_loop().create_future()
            if keys == {'values'}:
                values = message['values']
                validate(values, self.c.m)
                self.queue.put_nowait((self.owner, (values, done), 0))
            elif keys == {'assist'}:
                if not getattr(self.c, '_assisted_capability', False):
                    raise ValueError('assisted stand not available')
                op = message['assist']
                if op not in ('on', 'off', 'reset', 'status'):
                    raise ValueError('invalid assist op')
                self.queue.put_nowait((self.owner, ('assist', op, done), 0))
            else:
                raise ValueError('invalid request')
            # A client disconnect must not revoke its submitted command.
            reply = await asyncio.shield(done)
        except (ValueError, TypeError, OverflowError, TimeoutError, asyncio.QueueFull):
            self.record(values, False)
            reply = 'ERROR invalid or busy request'
        try:
            writer.write((reply + '\n').encode())
            await writer.drain()
        except (ConnectionError, BrokenPipeError):
            pass
        finally:
            writer.close()
            await writer.wait_closed()
