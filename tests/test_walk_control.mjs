// Animation mode stick -> walk mapping. Run: node tests/test_walk_control.mjs
import assert from 'node:assert/strict';
import { DEADZONE } from '../viewer/human/gamepad-map.js';
import { WALK_MAX_SPEED, WALK_MIN_SPEED, createWalkSession, walkFromStick } from '../viewer/human/walk-control.js';

const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`);

// Centered, inside the deadzone, or pulled DOWN: no walk.
for (const v of [0, DEADZONE, DEADZONE / 2, -0.5, -1, NaN]) assert.equal(walkFromStick(v).active, false, `v=${v}`);
// Just outside the deadzone: minimum speed; full up: maximum speed.
close(walkFromStick(DEADZONE + 1e-12).timeScale, WALK_MIN_SPEED);
close(walkFromStick(1).timeScale, WALK_MAX_SPEED);
// Monotonic: higher stick -> faster.
let prev = 0;
for (let v = DEADZONE + 0.01; v <= 1; v += 0.05) { const t = walkFromStick(v).timeScale; assert.ok(t > prev); prev = t; }
// Linear by default: halfway through the live range is halfway through the speed range.
close(walkFromStick(DEADZONE + (1 - DEADZONE) / 2).timeScale, (WALK_MIN_SPEED + WALK_MAX_SPEED) / 2);

// Session: right stick drives walk, left stick never does, release stops.
const s = createWalkSession();
assert.equal(s.sample().walk.active, false);
s.mouseDown('left', 0, 1);
assert.equal(s.sample().walk.active, false, 'left stick must not walk');
s.mouseUp('left');
s.mouseDown('right', 0, 1);
close(s.sample().walk.timeScale, WALK_MAX_SPEED);
s.mouseMove('right', 0, 0.3);
assert.ok(s.sample().walk.timeScale < WALK_MAX_SPEED);
s.mouseMove('right', 0, -1);
assert.equal(s.sample().walk.active, false, 'down does nothing');
s.mouseUp('right');
assert.equal(s.sample().walk.active, false, 'release stops');
// Gamepad: physical UP reports -1 on the raw axis; it must walk at full speed.
s.setGamepad({ connected: true, id: 'pad', mapping: 'standard', axes: [0, 0, 0, -1], buttons: [] });
close(s.sample().walk.timeScale, WALK_MAX_SPEED);
s.setGamepad({ connected: true, id: 'pad', mapping: 'standard', axes: [0, 0, 0, 1], buttons: [] });
assert.equal(s.sample().walk.active, false, 'gamepad down does nothing');
console.log('test_walk_control OK');
