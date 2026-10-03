import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { AUTHORITY_LEVELS, DEADZONE, DEFAULT_AUTHORITY, GAMEPAD_MAP, HUMAN_CMD_HZ, HUMAN_STALE_MS } from '../viewer/human/gamepad-map.js';
import { applyAuthority, applyDeadzone, mapNormalizedToCtrl, readGamepad, stickFromGamepadAxes } from '../viewer/human/normalize.js';
import { createMapper } from '../viewer/human/joint-mapper.js';
import { createSession } from '../viewer/human/session.js';
import { encodeHumanCmd, humanCmdMessage, startCommandPump } from '../viewer/human/transport.js';

const AXIS_Y_SIGN = GAMEPAD_MAP.AXIS_Y_SIGN;

const root = dirname(fileURLToPath(import.meta.url));
const order = JSON.parse(readFileSync(join(root, '../physics/model/actuator_order.json'), 'utf8'));
const actuators = order.map((a, id) => ({ id, name: a.name, joint: a.joint, ctrlrange: a.ctrlrange }));
const byName = Object.fromEntries(actuators.map((a) => [a.name, a]));

const passes = {};
const numbers = {};
function check(name, fn) {
  fn();
  passes[name] = true;
}

check('1_left_right_independent', () => {
  const s = createSession(actuators);
  s.select('right', 'hip_r');
  s.mouseDown('left', 0, 1);
  s.mouseDown('right', 1, 0);
  const { u } = s.sample();
  const lx = byName.shoulder_l_x_motor;
  const hy = byName.hip_r_y_motor;
  assert.notEqual(lx.id, hy.id);
  assert.ok(Math.abs(u[lx.id]) > 0);
  assert.ok(Math.abs(u[hy.id]) > 0);
  assert.equal(u[byName.shoulder_l_y_motor.id], 0);
  assert.equal(u[byName.hip_r_x_motor.id], 0);
  // Left command does not appear on the right shoulder, and vice versa.
  assert.equal(u[byName.shoulder_r_x_motor.id], 0);
  assert.equal(u[byName.hip_l_y_motor.id], 0);
  numbers.independent_left_shoulder_x = u[lx.id];
  numbers.independent_right_hip_y = u[hy.id];
});

check('2_release_to_zero', () => {
  const s = createSession(actuators);
  s.mouseDown('left', 0, 1);
  s.mouseDown('right', 1, 0.5);
  assert.ok(s.sample().u.some((v) => v !== 0));
  s.mouseUp('left');
  s.mouseUp('right');
  const { u } = s.sample();
  assert.ok(u.every((v) => v === 0));
  assert.equal(u.length, 33);
});

check('3_joint_switch_clears_previous', () => {
  const s = createSession(actuators);
  s.mouseDown('left', 0, 1);
  const before = s.sample().u;
  const shoulder = byName.shoulder_l_x_motor.id;
  const elbow = byName.elbow_l_motor.id;
  assert.ok(before[shoulder] !== 0);
  assert.equal(before[elbow], 0);
  assert.equal(s.cycle('left'), 'elbow_l');
  const after = s.sample().u;
  assert.equal(after[shoulder], 0);
  assert.ok(after[elbow] !== 0);
  numbers.elbow_after_switch = after[elbow];
});

check('4_z_modifier_release_clears_z', () => {
  const s = createSession(actuators);
  s.mouseDown('left', 1, 0);
  let sample = s.sample();
  const y = byName.shoulder_l_y_motor.id;
  const z = byName.shoulder_l_z_motor.id;
  assert.ok(sample.u[y] !== 0);
  assert.equal(sample.u[z], 0);
  assert.equal(sample.view.left.zActive, false);
  s.mouseZ('left', true);
  sample = s.sample();
  assert.equal(sample.u[y], 0);
  assert.ok(sample.u[z] !== 0);
  assert.equal(sample.view.left.zActive, true);
  numbers.z_while_held = sample.u[z];
  s.mouseZ('left', false);
  sample = s.sample();
  assert.equal(sample.u[z], 0);
  assert.ok(sample.u[y] !== 0);
  assert.equal(sample.view.left.zActive, false);
  // 1-DOF elbow has no fake Z actuator.
  s.select('left', 'elbow_l');
  s.mouseZ('left', true);
  s.mouseDown('left', 1, 0.7);
  sample = s.sample();
  const nonzero = sample.u.map((v, i) => v !== 0 ? i : -1).filter((i) => i >= 0);
  assert.deepEqual(nonzero, [byName.elbow_l_motor.id]);
  assert.equal(sample.view.left.zAvailable, false);
});

check('5_deadzone', () => {
  assert.equal(DEADZONE, 0.08);
  assert.equal(applyDeadzone(0.08), 0);
  assert.equal(applyDeadzone(-0.08), 0);
  assert.equal(applyDeadzone(0), 0);
  assert.equal(applyDeadzone(1), 1);
  assert.equal(applyDeadzone(-1), -1);
  const mid = 0.08 + (1 - 0.08) * 0.5;
  assert.ok(Math.abs(applyDeadzone(mid) - 0.5) < 1e-12);
  const s = createSession(actuators);
  s.mouseDown('left', 0, 0.08);
  assert.ok(s.sample().u.every((v) => v === 0));
  s.mouseDown('left', 0, 1);
  const full = s.sample().u[byName.shoulder_l_x_motor.id];
  const hi = byName.shoulder_l_x_motor.ctrlrange[1];
  assert.ok(Math.abs(full - hi * DEFAULT_AUTHORITY) < 1e-9);
  numbers.deadzone = DEADZONE;
  numbers.deadzone_mid_input = mid;
});

check('6_authority_scaling', () => {
  assert.equal(DEFAULT_AUTHORITY, 0.10);
  assert.deepEqual(AUTHORITY_LEVELS, [0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1]);
  const s = createSession(actuators);
  assert.equal(s.authority, 0.10);
  s.mouseDown('left', 0, 1);
  const hi = byName.shoulder_l_x_motor.ctrlrange[1];
  const id = byName.shoulder_l_x_motor.id;
  assert.ok(Math.abs(s.sample().u[id] - hi * 0.10) < 1e-9);
  // D-pad up steps one level and does not repeat while held.
  const buttons = Array.from({ length: 16 }, () => ({ pressed: false, value: 0 }));
  buttons[GAMEPAD_MAP.buttons.DpadUp] = { pressed: true, value: 1 };
  s.mouseUp('left');
  s.setGamepad({ connected: true, id: 'virtual', mapping: 'standard', axes: [0, 0, 0, 0], buttons });
  assert.equal(s.sample().authority, 0.20);
  assert.equal(s.sample().authority, 0.20);
  // Mouse buttons step the same list, clamped at the ends.
  while (s.authority < 1) s.stepAuthority(1);
  assert.equal(s.stepAuthority(1), 1);
  while (s.authority > 0.05) s.stepAuthority(-1);
  assert.equal(s.stepAuthority(-1), 0.05);
  numbers.authority_default = 0.10;
  numbers.authority_after_dpad_up = 0.20;
});

check('7_full_deflection_authority_times_ctrlrange', () => {
  const s = createSession(actuators);
  while (s.authority < 1) s.stepAuthority(1);
  s.mouseDown('left', -1, 1);
  s.mouseZ('left', true);
  const { u, zeroOutsideRange } = s.sample();
  assert.deepEqual(zeroOutsideRange, []);
  const cases = ['shoulder_l_x_motor', 'shoulder_l_z_motor'];
  for (const name of cases) {
    const a = byName[name];
    const end = name.endsWith('_x_motor') ? a.ctrlrange[1] : a.ctrlrange[0];
    // vertical +1 -> +hi; horizontal -1 with Z -> -lo end (lo is negative, -1 maps to lo)
    const expected = name.endsWith('_x_motor') ? a.ctrlrange[1] * 1 : a.ctrlrange[0] * 1;
    assert.ok(Math.abs(u[a.id] - expected) < 1e-9, `${name} ${u[a.id]} != ${expected}`);
    numbers[`full_${name}`] = u[a.id];
    numbers[`limit_${name}`] = expected;
  }
  // Default authority 10% reaches 10% of the real end, not a hard-coded torque table.
  const s2 = createSession(actuators);
  s2.mouseDown('left', 0, 1);
  for (const a of actuators) {
    if (a.name === 'shoulder_l_x_motor') {
      const got = s2.sample().u[a.id];
      assert.ok(Math.abs(got - a.ctrlrange[1] * 0.10) < 1e-9);
      numbers.shoulder_l_x_at_10pct = got;
      numbers.shoulder_l_x_ctrl_hi = a.ctrlrange[1];
    }
  }
  // Asymmetric and zero-outside formulas (not used by this inventory).
  assert.ok(Math.abs(mapNormalizedToCtrl(1, -20, 40) - 40) < 1e-12);
  assert.ok(Math.abs(mapNormalizedToCtrl(-1, -20, 40) - (-20)) < 1e-12);
  assert.ok(Math.abs(mapNormalizedToCtrl(0, -20, 40)) < 1e-12);
  assert.ok(Math.abs(mapNormalizedToCtrl(-1, 10, 20) - 10) < 1e-12);
  assert.ok(Math.abs(mapNormalizedToCtrl(1, 10, 20) - 20) < 1e-12);
  assert.ok(Math.abs(mapNormalizedToCtrl(0, 10, 20) - 15) < 1e-12);
  assert.ok(Math.abs(applyAuthority(60, 0.1, -60, 60) - 6) < 1e-12);
});

check('12_no_gamepad_does_not_throw', () => {
  assert.equal(readGamepad(undefined).connected, false);
  assert.equal(readGamepad(null).connected, false);
  assert.equal(readGamepad({}).connected, false);
  assert.equal(readGamepad({ getGamepads() { throw new Error('no device'); } }).connected, false);
  assert.equal(readGamepad({ getGamepads() { return [null, undefined]; } }).connected, false);
  const s = createSession(actuators);
  assert.doesNotThrow(() => s.sample());
  assert.equal(s.sample().gamepadConnected, false);
  assert.equal(s.sample().u.length, 33);
});

check('13_mapping_table_is_one_config', () => {
  const dir = join(root, '../viewer/human');
  const files = readdirSync(dir).filter((f) => f.endsWith('.js'));
  assert.ok(files.includes('gamepad-map.js'));
  const mapSrc = readFileSync(join(dir, 'gamepad-map.js'), 'utf8');
  assert.equal(GAMEPAD_MAP.buttons.L1, 4);
  assert.equal(GAMEPAD_MAP.buttons.R1, 5);
  assert.equal(GAMEPAD_MAP.buttons.L2, 6);
  assert.equal(GAMEPAD_MAP.buttons.R2, 7);
  assert.equal(GAMEPAD_MAP.buttons.DpadUp, 12);
  assert.equal(GAMEPAD_MAP.buttons.DpadDown, 13);
  assert.equal(GAMEPAD_MAP.buttons.DpadLeft, 14);
  assert.equal(GAMEPAD_MAP.buttons.DpadRight, 15);
  assert.equal(AXIS_Y_SIGN, -1);
  assert.match(mapSrc, /AXIS_Y_SIGN:\s*-1/);
  const leaked = [];
  for (const f of files) {
    if (f === 'gamepad-map.js') continue;
    const src = readFileSync(join(dir, f), 'utf8');
    if (/buttons\[\s*\d+\s*\]/.test(src) || /L1:\s*4/.test(src) || /DpadUp:\s*12/.test(src)) leaked.push(f);
    if (/\bWebSocket\b|ws:\/\//.test(src) && f !== 'transport.js' && f !== 'attach.js') leaked.push(`${f}:ws`);
  }
  assert.deepEqual(leaked, []);
  // Gamepad and mapper modules do not know the socket.
  for (const f of ['gamepad-map.js', 'normalize.js', 'joint-mapper.js', 'session.js']) {
    const src = readFileSync(join(dir, f), 'utf8');
    assert.equal(/\bWebSocket\b|human_cmd|ws:\/\//.test(src), false, f);
  }
  numbers.human_cmd_hz = HUMAN_CMD_HZ;
  numbers.human_stale_ms = HUMAN_STALE_MS;
  assert.equal(HUMAN_CMD_HZ, 50);
  assert.equal(HUMAN_STALE_MS, 200);
});

check('y_axis_sign_and_slot_conflict', () => {
  const up = stickFromGamepadAxes([0, -1, 0, 0], 'left');
  assert.equal(up.vertical, 1);
  const down = stickFromGamepadAxes([0, 1, 0, 0], 'left');
  assert.equal(down.vertical, -1);
  assert.equal(stickFromGamepadAxes([1, 0, -0.5, 0], 'right').horizontal, -0.5);

  const s = createSession(actuators);
  let sample = s.sample();
  assert.equal(sample.view.left.slot, 'A');
  assert.equal(sample.view.right.slot, 'B');
  assert.equal(sample.view.left.joint, 'shoulder_l');
  assert.equal(sample.view.right.joint, 'shoulder_r');
  assert.notEqual(sample.view.left.joint, sample.view.right.joint);

  const cycle = s.mapper.catalog.cycle;
  const covered = new Set();
  for (const joint of cycle) {
    const ids = joint.kind === 'hinge' ? [joint.actuator.id] : Object.values(joint.axes).map((a) => a.id);
    for (const id of ids) {
      assert.equal(covered.has(id), false, `ctrl ${id} grouped twice`);
      covered.add(id);
    }
  }
  assert.equal(covered.size, 33);

  // Both cyclers can reach every joint, including the opposite limb and the spine.
  assert.equal(s.select('right', 'ankle_r_ie'), 'ankle_r_ie');
  for (const joint of cycle) {
    if (joint.name === 'ankle_r_ie') continue;
    assert.equal(s.select('left', joint.name), joint.name, `slot A ${joint.name}`);
  }
  assert.equal(s.select('left', 'ankle_l_ie'), 'ankle_l_ie');
  for (const joint of cycle) {
    if (joint.name === 'ankle_l_ie') continue;
    assert.equal(s.select('right', joint.name), joint.name, `slot B ${joint.name}`);
  }

  // Selecting the joint (and therefore the ctrl indices) the other slot holds is refused.
  assert.equal(s.select('right', 'ankle_r_ie'), 'ankle_r_ie');
  assert.equal(s.select('left', 'neck'), 'neck');
  const parked = s.sample().view.right.joint;
  assert.notEqual(parked, 'neck');
  assert.equal(s.select('right', 'neck'), parked);
  assert.equal(s.sample().view.right.joint, parked);
  assert.equal(s.sample().view.left.joint, 'neck');

  // Cycling skips the held joint. shoulder_l -> elbow_l is held by B, so A lands on wrist_l_flex.
  assert.equal(s.select('left', 'shoulder_l'), 'shoulder_l');
  assert.equal(s.select('right', 'elbow_l'), 'elbow_l');
  assert.equal(s.cycle('left'), 'wrist_l_flex');
  // Wrap: B holds the first entry, A sits on the last, the next free is the second entry.
  assert.equal(s.select('right', 'shoulder_l'), 'shoulder_l');
  const last = cycle[cycle.length - 1].name;
  assert.equal(s.select('left', last), last);
  assert.equal(s.cycle('left'), cycle[1].name);
  assert.notEqual(cycle[1].name, 'shoulder_l');

  // Both sticks deflected at lumbar: B's selection is refused, so only A writes those ctrl indices.
  assert.equal(s.select('left', 'lumbar'), 'lumbar');
  assert.notEqual(s.select('right', 'lumbar'), 'lumbar');
  s.mouseDown('left', 0, 1);
  s.mouseDown('right', 0, -1);
  sample = s.sample();
  assert.equal(sample.view.left.joint, 'lumbar');
  assert.equal(sample.view.left.owns, true);
  assert.equal(sample.view.right.owns, true);
  assert.notEqual(sample.view.right.joint, 'lumbar');
  const lumbarIds = actuators.filter((a) => a.joint === 'lumbar').map((a) => a.id);
  const rightJoint = cycle.find((j) => j.name === sample.view.right.joint);
  const rightIds = rightJoint.kind === 'hinge'
    ? [rightJoint.actuator.id]
    : Object.values(rightJoint.axes).map((a) => a.id);
  for (const id of lumbarIds) assert.equal(rightIds.includes(id), false);
  for (const a of actuators) {
    if (sample.u[a.id] !== 0) assert.ok(lumbarIds.includes(a.id) || rightIds.includes(a.id), a.name);
  }
  const lumbarX = byName.lumbar_x_motor;
  assert.ok(Math.abs(sample.u[lumbarX.id] - lumbarX.ctrlrange[1] * DEFAULT_AUTHORITY) < 1e-9);
  assert.equal(sample.u[byName.lumbar_y_motor.id], 0);
  // Joint change clears the previous actuator. The refused slot never wrote it.
  assert.notEqual(s.cycle('left'), 'lumbar');
  const cleared = s.sample().u;
  assert.equal(cleared[lumbarX.id], 0);
  assert.equal(cleared[byName.lumbar_y_motor.id], 0);
  assert.equal(cleared[byName.lumbar_z_motor.id], 0);

  // Mouse drag beats a deflected gamepad stick on that stick.
  s.mouseUp('left');
  s.mouseUp('right');
  // B may still be holding shoulder_l from the wrap check, so free it before parking A there.
  assert.equal(s.select('right', 'shoulder_r'), 'shoulder_r');
  assert.equal(s.select('left', 'shoulder_l'), 'shoulder_l');
  s.mouseDown('left', 0, 0);
  const buttons = Array.from({ length: 16 }, () => ({ pressed: false, value: 0 }));
  s.setGamepad({ connected: true, id: 'virtual-ds', mapping: 'standard', axes: [0, -1, 0, 0], buttons });
  let owned = s.sample();
  assert.equal(owned.sticks.left.owner, 'mouse');
  assert.equal(owned.u[byName.shoulder_l_x_motor.id], 0);
  s.mouseUp('left');
  owned = s.sample();
  assert.equal(owned.sticks.left.owner, 'gamepad');
  assert.ok(owned.u[byName.shoulder_l_x_motor.id] > 0);
  // L1 rising cycles slot A once. B is on shoulder_r, so the next free joint is elbow_l.
  buttons[GAMEPAD_MAP.buttons.L1] = { pressed: true, value: 1 };
  s.setGamepad({ connected: true, id: 'virtual-ds', mapping: 'standard', axes: [0, 0, 0, 0], buttons });
  const cycled = s.sample();
  assert.equal(cycled.view.left.joint, 'elbow_l');
  assert.equal(s.sample().view.left.joint, 'elbow_l');
  assert.equal(cycled.view.right.joint, 'shoulder_r');
});

check('transport_contract', () => {
  const u = new Array(33).fill(0);
  u[3] = 1.25;
  const msg = humanCmdMessage(u, 1234);
  assert.deepEqual(Object.keys(msg), ['op', 'u', 't']);
  assert.equal(msg.op, 'human_cmd');
  assert.equal(msg.u.length, 33);
  assert.equal(msg.t, 1234);
  const encoded = JSON.parse(encodeHumanCmd(u, 50));
  assert.equal(encoded.op, 'human_cmd');
  assert.equal(encoded.u[3], 1.25);
  assert.throws(() => humanCmdMessage(u.slice(0, 32), 1));
  const sent = [];
  const socket = { readyState: 1, send(data) { sent.push(data); } };
  let ticks = 0;
  const stop = startCommandPump(() => socket, () => (ticks++ ? null : u), {
    hz: 50,
    now: () => 99,
    setIntervalFn(fn) { fn(); return 1; },
    clearIntervalFn() {},
  });
  stop();
  assert.equal(sent.length, 1);
  assert.equal(JSON.parse(sent[0]).t, 99);
  assert.equal(JSON.parse(sent[0]).op, 'human_cmd');
});

check('14_human_js_does_not_mention_qpos_qvel', () => {
  const dir = join(root, '../viewer/human');
  for (const f of readdirSync(dir)) {
    const src = readFileSync(join(dir, f), 'utf8');
    assert.equal(/\bqpos\b|\bqvel\b/.test(src), false, f);
  }
});

const summary = { passes, numbers, all_pass: Object.values(passes).every(Boolean) };
console.log('HUMAN_RESULT ' + JSON.stringify(summary));
if (!summary.all_pass) process.exit(1);
