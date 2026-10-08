// Animation mode locomotion controller (EXP002.1-B). Run: node tests/test_locomotion.mjs
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  DEFAULT_SPEEDS, LOCOMOTION_DEFAULTS, createMovementController, keyboardInput, keyboardVector,
  maxSpeed, selectLocomotion, stickInput, wrapAngle,
} from '../viewer/human/locomotion.js';
import { createWalkSession } from '../viewer/human/walk-control.js';

const close = (a, b, eps = 1e-6, msg = '') => assert.ok(Math.abs(a - b) <= eps, `${msg} ${a} != ${b}`);
const manifest = JSON.parse(readFileSync(new URL('../assets/Xandra_Animated.manifest.json', import.meta.url)));
const speeds = Object.fromEntries(Object.keys(DEFAULT_SPEEDS).map((n) => [n, manifest.clips[n].natural_ground_speed_mps]));
// Built-in fallback speeds match the authoritative manifest.
for (const n of Object.keys(DEFAULT_SPEEDS)) close(DEFAULT_SPEEDS[n], speeds[n], 1e-9, n);

// Keyboard: diagonals are not faster; opposite keys cancel; Shift/Caps run.
close(Math.hypot(...Object.values(keyboardVector({ w: true, d: true }))), 1);
assert.deepEqual(keyboardVector({ w: true, s: true }), { x: 0, y: 0 });
assert.equal(keyboardInput({}), null);
assert.equal(keyboardInput({ w: true }).gait, 0);
assert.equal(keyboardInput({ w: true }, { shift: true }).gait, 1);
assert.equal(keyboardInput({ w: true }, { capsLock: true }).gait, 1);

// Stick: radial deadzone, direction kept, magnitude <= 1, outer zone runs.
assert.equal(stickInput(0.05, 0.05), null);
const half = stickInput(0, 0.5);
assert.ok(half.y > 0 && half.x === 0 && half.gait === 0);
const diag = stickInput(Math.SQRT1_2, Math.SQRT1_2);
close(Math.hypot(diag.x, diag.y), 1, 1e-9, 'diag magnitude');
assert.equal(diag.gait, 1, 'rim = run');
assert.ok(stickInput(0, 0.9, { gamma: 2 }).y < stickInput(0, 0.9).y, 'expo shapes magnitude');

// Speeds come from the manifest.
close(maxSpeed(false, 0, speeds), 1.442); close(maxSpeed(false, 1, speeds), 3.016); close(maxSpeed(true, 0, speeds), 0.647);

// Locomotion selection: timeScale = |speed| / natural; hysteresis; Walk_Back for backward.
let sel = selectLocomotion('Idle', 1.442, speeds); assert.equal(sel.clip, 'Walk_Fwd'); close(sel.timeScale, 1);
sel = selectLocomotion('Walk', -0.647, speeds); assert.equal(sel.clip, 'Walk_Back'); close(sel.timeScale, 1);
sel = selectLocomotion('Walk', 3.016, speeds); assert.equal(sel.state, 'Run'); close(sel.timeScale, 1);
assert.equal(selectLocomotion('Run', 2.0, speeds).state, 'Run', 'run hysteresis');
assert.equal(selectLocomotion('Walk', 2.0, speeds).state, 'Walk');
assert.equal(selectLocomotion('Walk', 0.01, speeds).state, 'Idle');
assert.equal(selectLocomotion('Idle', 0.06, speeds).state, 'Idle', 'idle hysteresis');
// Graceful fallbacks when clips are missing.
assert.equal(selectLocomotion('Walk', 3, speeds, undefined, new Set(['Idle', 'Walk_Fwd'])).clip, 'Walk_Fwd');
sel = selectLocomotion('Walk', -0.5, speeds, undefined, new Set(['Idle', 'Walk_Fwd']));
assert.equal(sel.clip, 'Walk_Fwd'); assert.ok(sel.timeScale < 0, 'reverse playback fallback');
assert.equal(selectLocomotion('Idle', 0, speeds, undefined, new Set(['Walk_Fwd'])).clip, null);

// Controller: world-space translation with delta time, smooth accel/decel.
const run = (c, seconds, input, fwd = { x: 0, z: 1 }, dt = 1 / 60) => { let s; for (let t = 0; t < seconds - 1e-9; t += dt) s = c.update(dt, input, fwd); return s; };
let c = createMovementController({ speeds });
let s = run(c, 3, { x: 0, y: 1, gait: 0, source: 'keyboard' });
close(s.speed, 1.442, 1e-9, 'steady walk speed'); assert.ok(s.position.z > 3.3, `moved forward ${s.position.z}`);
close(s.yaw, 0, 1e-9);
// Frame-rate independence: 30 fps vs 120 fps end positions agree.
const c30 = createMovementController({ speeds }), c120 = createMovementController({ speeds });
const p30 = run(c30, 2, { x: 0, y: 1, gait: 0 }, undefined, 1 / 30).position.z, p120 = run(c120, 2, { x: 0, y: 1, gait: 0 }, undefined, 1 / 120).position.z;
close(p30, p120, 0.03, 'dt independence');
// Release: decelerates smoothly (no instant stop), then rests.
const r1 = c.update(1 / 60, null);
assert.ok(r1.speed > 1.3 && r1.speed < 1.442, 'smooth decel');
s = run(c, 1, null); assert.equal(s.speed, 0);
// Run with Shift: reaches 3.016.
s = run(c, 3, { x: 0, y: 1, gait: 1 }); close(s.speed, 3.016, 1e-9);
// Backward: keeps facing, negative speed, moves toward -forward.
c = createMovementController({ speeds });
s = run(c, 3, { x: 0, y: -1, gait: 0 }, { x: 0, z: -1 });
assert.ok(s.backward); close(s.speed, -0.647, 1e-9); close(Math.abs(wrapAngle(s.yaw - Math.PI)), 0, 1e-3, 'faces camera-forward (-Z)');
assert.ok(s.position.z > 1.4, `walked back toward +Z ${s.position.z}`);
// Lateral: turns to face the side and walks there.
c = createMovementController({ speeds });
s = run(c, 3, { x: 1, y: 0, gait: 0 }, { x: 0, z: 1 });
assert.ok(!s.backward); close(s.speed, 1.442, 1e-6); assert.ok(s.position.x < -2, `right of +Z forward is -X: ${s.position.x}`);
// Diagonal is not faster than straight.
c = createMovementController({ speeds });
s = run(c, 3, keyboardInput({ w: true, d: true })); close(s.speed, 1.442, 1e-6, 'diagonal speed');
assert.ok(Math.abs(LOCOMOTION_DEFAULTS.bounds) > 0);

// Session: LEFT stick produces the move input, the legacy RIGHT stick still walks.
const ws = createWalkSession();
assert.equal(ws.sample().move, null);
ws.mouseDown('left', -1, 0);
const mv = ws.sample().move; assert.ok(mv.x < 0 && mv.y === 0, 'left stick left = move left');
assert.equal(ws.sample().walk.active, false);
ws.setPointerType('touch'); assert.equal(ws.sample().move.source, 'touch');
ws.mouseUp('left'); assert.equal(ws.sample().move, null, 'release -> no input');
ws.mouseDown('right', 0, 1); assert.equal(ws.sample().walk.active, true); ws.mouseUp('right');
console.log('test_locomotion OK');
