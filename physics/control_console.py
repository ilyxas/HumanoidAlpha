"""Developer console. MuJoCo writes happen on the physics loop only.

Controller.handle, Controller.tick, and harness ingest_human_cmd are the writers.
The WebSocket handler only stages human_cmd and does not touch MuJoCo state.
"""
from __future__ import annotations
import argparse
from contextlib import AsyncExitStack
from actuator_runtime import ActuatorAPI, default_socket
import asyncio
import json
import math
import time
from pathlib import Path
import numpy as np
import mujoco
import websockets
from agent_api.env import HumanoidEnv
from diagnostic_api.diag import HumanoidDiag
from bridge_full.mapping_full import BODY_TO_BONE, DERIVED_BONE_SOURCES, extract_full_pose
from collision_guard import PositionCollisionGuard
from assisted_stand import AssistedStandController
from harness import HarnessSupport

# Zero the human actuator vector if the browser stops streaming. Harness wrench
# is independent and is not cleared by this timeout. 200 ms is inside 150–250 ms.
HUMAN_CMD_STALE_S = 0.200

SIZES = {0: (7, 6), 1: (4, 3), 2: (1, 1), 3: (1, 1)}
TYPES = {0: 'free', 1: 'ball', 2: 'slide', 3: 'hinge'}

def region(body):
    if body == 'pelvis': return 'Root'
    if body == 'head': return 'Head'
    if body.startswith('torso'): return 'Torso'
    side = 'Left' if body.endswith('_l') else 'Right'
    return side + (' arm' if any(s in body for s in ('arm', 'hand')) else ' leg')

def finite(value, size):
    a = np.asarray(value, dtype=float).reshape(-1)
    if len(a) != size or not np.isfinite(a).all():
        raise ValueError(f'Expected {size} finite numbers')
    return a

def quaternion(value, limit=math.pi):
    q = finite(value, 4)
    n = np.linalg.norm(q)
    if n < 1e-9: raise ValueError('Zero quaternion is invalid')
    q = q / n
    if q[0] < 0: q = -q
    angle = 2 * math.atan2(np.linalg.norm(q[1:]), q[0])
    if angle > limit + 1e-8: raise ValueError('Rotation exceeds joint cone limit')
    return q

class Controller:
    def __init__(self, assisted_stand=False, harness=False):
        self.env = HumanoidEnv()
        self.diag = HumanoidDiag(self.env)
        self.m, self.d = self.env.model, self.env.data
        self.position_guard = PositionCollisionGuard(self.m)
        self.mode = 'kinematic'
        self.paused = True
        self.revision = 0
        self.assisted = False
        # Capability always available in console; --assisted-stand only auto-enables on serve.
        self.assist = AssistedStandController(self.m)
        # HARNESS is a separate launcher mode. Disabled means the step loop never
        # calls it, so RAW/experiment leaves xfrc_applied alone.
        self.harness = HarnessSupport(self.m)
        self.u_cmd = np.zeros(self.m.nu, dtype=float)
        # Latest human vector. Applied to ctrl only while the harness is enabled.
        # The WS handler only stages the dict; ingest/poll run on this loop.
        self.human_u = np.zeros(self.m.nu, dtype=float)
        self._human_stream = False
        self._human_mono = None
        self._staged_human = None
        self._assisted_capability = True
        self._auto_assist = bool(assisted_stand)
        self._auto_harness = bool(harness)
        self.env.reset()
        self.inventory = self.discover()

    def discover(self):
        m = self.m
        bodies = []
        for i in range(m.nbody):
            name = m.body(i).name
            bones = ([BODY_TO_BONE[name]] if name in BODY_TO_BONE else [])
            for bone, source in DERIVED_BONE_SOURCES.items():
                if source == name or (isinstance(source, tuple) and name in source[1:3]):
                    bones.append(bone)
            bodies.append(dict(id=i, name=name, parent=int(m.body_parentid[i]), visual_bones=bones))
        joints = []
        for i in range(m.njnt):
            b = bodies[int(m.jnt_bodyid[i])]
            t = int(m.jnt_type[i])
            nq, nv = SIZES[t]
            joints.append(dict(id=i, name=m.joint(i).name, type=TYPES[t], body=b['name'],
                body_id=b['id'], region=region(b['name']), axis=m.jnt_axis[i].tolist(),
                limited=bool(m.jnt_limited[i]), range=m.jnt_range[i].tolist(),
                qpos_address=int(m.jnt_qposadr[i]), qvel_address=int(m.jnt_dofadr[i]),
                nq=nq, nv=nv, visual_bones=b['visual_bones'],
                mapping_status='configured — anatomical validation required' if b['visual_bones'] else 'UNMAPPED'))
        actuators = []
        for i in range(m.nu):
            jid = int(m.actuator_trnid[i, 0])
            joint_transmission = int(m.actuator_trntype[i]) in (0, 1)
            j = joints[jid] if joint_transmission and 0 <= jid < len(joints) else None
            # This console accepts torque motors only; other models remain inspectable.
            motor = int(m.actuator_dyntype[i]) == 0 and int(m.actuator_gaintype[i]) == 0 and int(m.actuator_biastype[i]) == 0
            bounded = bool(m.actuator_ctrllimited[i]) and np.isfinite(m.actuator_ctrlrange[i]).all()
            actuators.append(dict(id=i, name=m.actuator(i).name, joint_id=jid if j else None,
                joint=j['name'] if j else None, region=j['region'] if j else 'Other',
                body=j['body'] if j else None, visual_bones=j['visual_bones'] if j else [],
                ctrlrange=m.actuator_ctrlrange[i].tolist(), limited=bool(m.actuator_ctrllimited[i]),
                gear=m.actuator_gear[i].tolist(), controllable=bool(j and motor and bounded),
                transmission=int(m.actuator_trntype[i])))
        return dict(bodies=bodies, joints=joints, actuators=actuators,
            nq=int(m.nq), nv=int(m.nv), nu=int(m.nu), timestep=float(m.opt.timestep),
            reference_qpos=m.qpos0.tolist(),
            assisted_capability=True)

    def stop(self):
        self.d.ctrl[:] = 0
        self.u_cmd[:] = 0
        self.paused = True
        self.revision += 1

    def _disable_assist(self):
        """Turn off assist without rewriting qpos; leave ctrl as last u_cmd."""
        self.assisted = False
        if self.assist is not None:
            self.assist.enabled = False

    def handle(self, cmd):
        op = cmd.get('op')
        if op == 'reset':
            self._disable_assist()
            self.u_cmd[:] = 0
            self.env.reset()
            self.mode, self.paused = 'kinematic', True
        elif op == 'mode':
            value = cmd.get('value')
            if value not in ('kinematic', 'dynamic', 'assisted'):
                raise ValueError('Unknown mode')
            if value == 'assisted':
                # Third UI mode: dynamic physics + assist ON; seed stand once; auto-resume.
                self.mode = 'dynamic'
                self.d.qvel[:] = 0
                self.d.qacc_warmstart[:] = 0
                self.set_assisted(True)  # reset_to_stand + u_cmd=0 + compute
                self.paused = False
            elif value == 'kinematic':
                self._disable_assist()
                self.u_cmd[:] = 0
                self.mode, self.paused = 'kinematic', True
                self.d.ctrl[:] = 0
                self.d.qvel[:] = 0
                self.d.qacc_warmstart[:] = 0
                mujoco.mj_forward(self.m, self.d)
            else:  # dynamic / Torque — RAW; disable assist if was on
                self._disable_assist()
                self.u_cmd[:] = 0
                self.mode, self.paused = 'dynamic', True
                self.d.ctrl[:] = 0
                self.d.qvel[:] = 0
                self.d.qacc_warmstart[:] = 0
                mujoco.mj_forward(self.m, self.d)
        elif op == 'assist':
            # WS side-channel without actuator API: value on|off|reset
            val = cmd.get('value')
            if val == 'on':
                if self.mode != 'dynamic':
                    self.mode = 'dynamic'
                self.set_assisted(True)
                self.paused = False
            elif val == 'off':
                self.set_assisted(False)
            elif val == 'reset':
                self.assist_reset()
                if self.mode != 'dynamic':
                    self.mode = 'dynamic'
                self.paused = False
            else:
                raise ValueError('assist value must be on|off|reset')
        elif op == 'pause': self.paused = True
        elif op == 'resume':
            if self.mode != 'dynamic': raise ValueError('Resume requires dynamic mode')
            self.paused = False
        elif op == 'zero': self.stop()
        elif op == 'joint':
            if self.mode != 'kinematic': raise ValueError('Position commands require kinematic mode')
            jid = cmd.get('id')
            if type(jid) is not int or not 0 <= jid < self.m.njnt: raise ValueError('Invalid joint ID')
            j = self.inventory['joints'][jid]
            a, n = j['qpos_address'], j['nq']
            if j['type'] in ('hinge', 'slide'):
                value = finite(cmd['value'], 1)
                if j['limited'] and not j['range'][0] <= value[0] <= j['range'][1]:
                    raise ValueError('Position exceeds joint limits')
                if not j['limited'] and abs(value[0]) > 100: raise ValueError('Console bound exceeded')
            elif j['type'] == 'ball':
                value = quaternion(cmd['value'], j['range'][1] if j['limited'] else math.pi)
            else:
                value = finite(cmd['value'], 7)
                if np.max(np.abs(value[:3])) > 10: raise ValueError('Root workspace bound: ±10 m')
                value[3:] = quaternion(value[3:])
            target = self.d.qpos.copy()
            target[a:a+n] = value
            self.position_guard.validate(self.d.qpos, target)
            self.d.qpos[:] = target
            self.d.qvel[:] = 0
            mujoco.mj_forward(self.m, self.d)
        elif op == 'torque':
            if self.mode != 'dynamic': raise ValueError('Torque commands require dynamic mode')
            aid = cmd.get('id')
            if type(aid) is not int or not 0 <= aid < self.m.nu: raise ValueError('Invalid actuator ID')
            a = self.inventory['actuators'][aid]
            if not a['controllable']: raise ValueError('Unsupported or unbounded actuator')
            v = float(finite(cmd['value'], 1)[0])
            if not a['ctrlrange'][0] <= v <= a['ctrlrange'][1]: raise ValueError('Torque exceeds range')
            if self.assisted and self.assist is not None:
                # Sliders write feedforward intent; tick applies PD+COM on top.
                self.u_cmd[aid] = v
                self.d.ctrl[:] = self.assist.compute(self.m, self.d, self.u_cmd)
            else:
                self.d.ctrl[aid] = v
                self.u_cmd[aid] = v
        else: raise ValueError('Unknown command')
        self.revision += 1

    def set_assisted(self, enabled: bool):
        """Enable/disable ASSISTED layer. Enabling reseeds to stand once."""
        if not self._assisted_capability or self.assist is None:
            raise ValueError('Assisted stand not available')
        if enabled:
            self.assist.reset_to_stand(self.m, self.d)
            self.assist.enabled = True
            self.assisted = True
            self.u_cmd[:] = 0
            self.d.ctrl[:] = self.assist.compute(self.m, self.d, self.u_cmd)
        else:
            self.assisted = False
            self.assist.enabled = False
            # Hand control back to last RAW command without rewriting qpos.
            self.d.ctrl[:] = self.u_cmd
        self.revision += 1

    def assist_reset(self):
        """Re-seed stand pose (qpos write once) and clear u_cmd."""
        if not self._assisted_capability or self.assist is None:
            raise ValueError('Assisted stand not available')
        self.assist.reset_to_stand(self.m, self.d)
        self.u_cmd[:] = 0
        self.assisted = True
        self.assist.enabled = True
        self.d.ctrl[:] = self.assist.compute(self.m, self.d, self.u_cmd)
        self.revision += 1

    def enable_harness(self, seed=True):
        """Turn on the pelvis harness. Optional stand seed is once, not per tick.

        Does not change actuator ctrl ranges or gravity. After the seed, u_cmd
        still reaches the actuators unchanged; the harness only adds xfrc.
        """
        if self.assisted:
            raise ValueError('Disable assisted stand before enabling the harness')
        if seed:
            self.harness.seed_stand(self.m, self.d)
            self.u_cmd[:] = 0
        self.harness.enable()
        self.revision += 1

    def stage_human_cmd(self, cmd):
        """Remember the latest human_cmd. Does not touch MuJoCo state."""
        self._staged_human = cmd

    def take_staged_human(self):
        cmd = self._staged_human
        self._staged_human = None
        return cmd

    def ingest_human_cmd(self, cmd, now=None):
        """Copy a complete actuator vector into ctrl when the harness is on.

        Ignored in RAW, debug, and assisted (harness disabled), so those paths
        do not require human_cmd and do not change ctrl. Does not assign
        generalized position or velocity. Out-of-range values are clipped
        to the model ctrlrange.
        A bad message leaves the previous human vector unchanged.
        """
        if not self.harness.enabled:
            return False
        if not isinstance(cmd, dict):
            return False
        u = cmd.get('u')
        if not isinstance(u, list) or len(u) != int(self.m.nu):
            return False
        out = np.empty(self.m.nu, dtype=float)
        for i, v in enumerate(u):
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                return False
            fv = float(v)
            if not math.isfinite(fv):
                return False
            if bool(self.m.actuator_ctrllimited[i]):
                lo = float(self.m.actuator_ctrlrange[i, 0])
                hi = float(self.m.actuator_ctrlrange[i, 1])
                fv = min(max(fv, lo), hi)
            out[i] = fv
        self.human_u[:] = out
        self.u_cmd[:] = out
        self.d.ctrl[:] = out
        self._human_stream = True
        self._human_mono = time.perf_counter() if now is None else float(now)
        return True

    def poll_human_stale(self, now=None):
        """If the human stream is older than HUMAN_CMD_STALE_S, zero it.

        The harness stays enabled. Returns True when the vector was zeroed.
        """
        if not self._human_stream or not self.harness.enabled:
            return False
        now = time.perf_counter() if now is None else float(now)
        last = self._human_mono
        if last is None or (now - last) > HUMAN_CMD_STALE_S:
            self.human_u[:] = 0
            self.u_cmd[:] = 0
            self.d.ctrl[:] = 0
            return True
        return False

    def assist_status(self):
        base = dict(
            capability=bool(self._assisted_capability),
            assisted=bool(self.assisted),
            mode=self.mode,
            paused=self.paused,
            revision=self.revision,
        )
        if self.assist is not None:
            base['diagnostics'] = self.assist.diagnostics
        return base

    def tick(self):
        if self.mode == 'dynamic' and not self.paused:
            if self.assisted and self.assist is not None:
                self.d.ctrl[:] = self.assist.compute(self.m, self.d, self.u_cmd)
            elif self.harness.enabled and self._human_stream:
                # Latest human vector, not a latched torque. Harness wrench below
                # does not read or write ctrl. No qpos/qvel write on this path.
                self.d.ctrl[:] = self.human_u
                self.u_cmd[:] = self.human_u
            # RAW/experiment: harness.enabled is false, so this block does not run
            # and xfrc_applied is left at zero. HARNESS mode sets the wrench here,
            # before mj_step, without writing qpos/qvel.
            if self.harness.enabled:
                self.harness.apply(self.m, self.d)
            mujoco.mj_step(self.m, self.d)
            if not np.isfinite(self.d.qpos).all() or not np.isfinite(self.d.qvel).all():
                self.env.reset()
                self.stop()
                raise ValueError('Nonfinite physics state; reset and paused')
            # mj_step may leave world transforms at the preceding integration state.
            mujoco.mj_forward(self.m, self.d)

    def snapshot(self):
        pose = extract_full_pose(self.env.observe(), self.diag.body_xpos(), self.diag.body_xquat())
        pose['apply_root_translate'] = True
        return dict(type='pose', sim_time=float(self.d.time), t_send_ms=int(time.time()*1000),
            pose=pose, state=dict(mode=self.mode, paused=self.paused, revision=self.revision,
                assisted=bool(self.assisted),
                qpos=self.d.qpos.tolist(), qvel=self.d.qvel.tolist(), ctrl=self.d.ctrl.tolist(),
                u_cmd=self.u_cmd.tolist(),
                joint_anchors=self.d.xanchor.tolist(), joint_axes=self.d.xaxis.tolist(),
                body_positions=self.d.xpos.tolist(), ncon=int(self.d.ncon)))

async def serve(port=8766, actuator_socket=None, actuator_log=None, assisted_stand=False, harness=False):
    if assisted_stand and harness:
        raise RuntimeError('harness and assisted stand cannot both auto-start')
    c = Controller(assisted_stand=assisted_stand, harness=harness)
    queue = asyncio.Queue(maxsize=128)
    clients = set()
    api_owner = object() if actuator_socket else None
    owner = api_owner
    stop_requested = False
    epoch = 0
    async def send(ws, msg):
        try: await asyncio.wait_for(ws.send(json.dumps(msg, allow_nan=False)), timeout=0.5)
        except (websockets.ConnectionClosed, TimeoutError): pass
    async def handler(ws):
        nonlocal owner, stop_requested, epoch
        clients.add(ws)
        if owner is None: owner = ws
        await send(ws, dict(type='inventory', inventory=c.inventory, can_control=owner is ws))
        try:
            async for raw in ws:
                try:
                    cmd = json.loads(raw)
                    if not isinstance(cmd, dict): raise ValueError('Expected command object')
                    # human_cmd is staged, not queued, so a 50 Hz stream cannot
                    # crowd out actuator-API or console commands. No MuJoCo write here.
                    if cmd.get('op') == 'human_cmd':
                        c.stage_human_cmd(cmd)
                        continue
                    if ws is not owner: raise ValueError('Read-only client; reconnect after controller closes')
                    # Epoch barriers reject stale pre-reset commands. No state writes here.
                    if cmd.get('op') in ('reset', 'zero'):
                        epoch += 1
                        while not queue.empty(): queue.get_nowait()
                    queue.put_nowait((ws, cmd, epoch))
                except (ValueError, TypeError, asyncio.QueueFull) as e:
                    await send(ws, dict(type='error', message=str(e) or 'Command queue full'))
        finally:
            clients.discard(ws)
            if owner is ws:
                owner = None
                epoch += 1
                stop_requested = True
                while not queue.empty(): queue.get_nowait()

    async with AsyncExitStack() as stack:
        api = None
        if actuator_socket:
            api = ActuatorAPI(c, queue, api_owner, actuator_socket, actuator_log)
            await stack.enter_async_context(api)
            c.handle({'op': 'mode', 'value': 'dynamic'})
            if assisted_stand or getattr(c, '_auto_assist', False):
                c.set_assisted(True)
            if harness or getattr(c, '_auto_harness', False):
                # Stand seed once, then only external pelvis wrench each tick.
                c.enable_harness(seed=True)
            c.handle({'op': 'resume'})
        await stack.enter_async_context(websockets.serve(handler, '127.0.0.1', port, max_size=16384,
            origins=[None, f'http://127.0.0.1:8788', f'http://localhost:8788'], ping_interval=10, ping_timeout=10))
        print(f'Control WebSocket: ws://127.0.0.1:{port}', flush=True)
        last = time.perf_counter()
        broadcast = last
        accumulator = 0.0
        while True:
            now = time.perf_counter()
            elapsed, last = min(now-last, 0.05), now
            if stop_requested:
                c.stop()
                stop_requested = False
            staged = c.take_staged_human()
            if staged is not None:
                c.ingest_human_cmd(staged)
            if c.harness.enabled:
                c.poll_human_stale()
            for _ in range(128):
                if queue.empty(): break
                ws, cmd, command_epoch = queue.get_nowait()
                if api is not None and ws is api_owner:
                    api.apply(cmd)
                    continue
                if ws is not owner or command_epoch != epoch: continue
                try:
                    c.handle(cmd)
                    await send(ws, dict(type='ack', id=cmd.get('request_id'), revision=c.revision))
                except (ValueError, KeyError, TypeError, OverflowError) as e:
                    await send(ws, dict(type='error', message=str(e)))
            if c.mode == 'dynamic' and not c.paused:
                accumulator += elapsed
                while accumulator >= c.m.opt.timestep:
                    c.tick()
                    accumulator -= c.m.opt.timestep
            else: accumulator = 0.0
            if now - broadcast >= 1/30:
                msg = c.snapshot()
                await asyncio.gather(*(send(ws, msg) for ws in tuple(clients)))
                broadcast = now
            await asyncio.sleep(0.002)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--inventory', type=Path)
    parser.add_argument('--actuator-api', action='store_true', help='Reserve runtime control for local actuator CLI')
    parser.add_argument('--actuator-socket', type=Path, default=default_socket())
    parser.add_argument('--actuator-log', type=Path, default=Path(__file__).resolve().parents[1] / 'reports/actuator_commands.jsonl')
    parser.add_argument('--assisted-stand', action='store_true',
                        help='Auto-enable ASSISTED stand on start (unadvertised assisted launch; capability always on)')
    parser.add_argument('--harness', action='store_true',
                        help='HARNESS mode: observation runtime plus pelvis xfrc support (not a pose weld)')
    args = parser.parse_args()
    if args.inventory:
        args.inventory.write_text(json.dumps(Controller().inventory, indent=2))
    else:
        if args.assisted_stand and args.harness:
            parser.error('--harness and --assisted-stand are mutually exclusive')
        if args.assisted_stand and not args.actuator_api:
            parser.error('--assisted-stand requires --actuator-api')
        if args.harness and not args.actuator_api:
            parser.error('--harness requires --actuator-api')
        asyncio.run(serve(args.port, args.actuator_socket if args.actuator_api else None,
                          args.actuator_log, assisted_stand=args.assisted_stand, harness=args.harness))
