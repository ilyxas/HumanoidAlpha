/**
 * Animation mode locomotion (no physics, no three.js). Pure functions so the
 * same logic runs in the browser and in `node tests/test_locomotion.mjs`.
 *
 * One movement controller is fed by every input source (keyboard, on-screen
 * sticks / touch, gamepad). Sources only produce a MoveInput:
 *   { x, y, gait, source }   x = right (+) / left (-), y = forward (+) / back (-),
 *                            |(x, y)| <= 1 (diagonals never exceed 1),
 *                            gait 0 = walk .. 1 = run.
 * The controller turns that into a world-space position / yaw / signed speed,
 * integrated with delta time. The animation clips are in place; the speeds
 * below come from assets/Xandra_Animated.manifest.json (natural_ground_speed_mps).
 *
 * Conventions (from the GLB): +Y up, the character faces +Z at yaw 0,
 * facing = (sin yaw, 0, cos yaw). Input is relative to a ground-plane
 * reference "forward" (the camera's view direction projected on the floor).
 */
import { DEADZONE } from './gamepad-map.js';
import { applyDeadzone, applyExpo } from './normalize.js';

/** Fallback values = the manifest of the EXP002.1-A asset; used only if the manifest cannot be read. */
export const DEFAULT_SPEEDS = Object.freeze({ Walk_Fwd: 1.442, Run_Fwd: 3.016, Walk_Back: 0.647 });

export const LOCOMOTION_DEFAULTS = Object.freeze({
  accel: 2.5,             // m/s^2 when speeding up
  decel: 3.5,             // m/s^2 when slowing down / on release
  turnRate: 9,            // 1/s, exponential approach to the target yaw
  maxTurnSpeed: 7,        // rad/s cap
  backRunFactor: 1.5,     // Shift/Caps while walking backwards: up to 1.5 x Walk_Back speed
  backEnterDeg: 120,      // input this far from "forward" -> backward walk (facing kept)
  backExitDeg: 100,       // hysteresis
  idleEnter: 0.05,        // m/s: below this the controller is Idle
  idleExit: 0.08,
  runEnter: 2.1,          // m/s: Walk -> Run
  runExit: 1.9,           // m/s: Run -> Walk
  stickRunZone: 0.8,      // stick drive above this blends walk -> run (outer 20 % of travel)
  bounds: 95,             // |x|, |z| clamp in metres (floor is 200 x 200 m)
});

const TAU = Math.PI * 2;
export function wrapAngle(a) { return ((a + Math.PI) % TAU + TAU) % TAU - Math.PI; }
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => (Number.isFinite(v) ? v : 0);

/** WASD -> unit-disc vector. Opposite keys cancel; diagonals are normalized (no sqrt(2) boost). */
export function keyboardVector(keys = {}) {
  const x = (keys.d ? 1 : 0) - (keys.a ? 1 : 0);
  const y = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  const m = Math.hypot(x, y);
  return m > 0 ? { x: x / m, y: y / m } : { x: 0, y: 0 };
}

/**
 * Keyboard -> MoveInput. Shift (held) or Caps Lock (persistent) runs.
 * Returns null when no movement key is held.
 */
export function keyboardInput(keys = {}, { shift = false, capsLock = false } = {}) {
  const v = keyboardVector(keys);
  if (v.x === 0 && v.y === 0) return null;
  return { x: v.x, y: v.y, gait: shift || capsLock ? 1 : 0, source: 'keyboard' };
}

/**
 * Stick (horizontal +right, vertical +up, already clamped to the unit disc) -> MoveInput.
 * Radial deadzone + the project's expo curve on the magnitude, so direction is kept exactly.
 * drive <= stickRunZone: walk, speed proportional to drive; above: walk -> run blend.
 */
export function stickInput(horizontal, vertical, { gamma = 1, dz = DEADZONE, runZone = LOCOMOTION_DEFAULTS.stickRunZone, source = 'touch' } = {}) {
  const h = finite(horizontal), v = finite(vertical);
  const m = Math.min(1, Math.hypot(h, v));
  const drive = applyExpo(applyDeadzone(m, dz), gamma);
  if (!(drive > 0)) return null;
  const amount = Math.min(1, drive / runZone);
  const gait = drive > runZone ? clamp((drive - runZone) / (1 - runZone), 0, 1) : 0;
  return { x: (h / m) * amount, y: (v / m) * amount, gait, drive, source };
}

/** Max ground speed for a direction/gait, from the manifest speeds. */
export function maxSpeed(backward, gait, speeds = DEFAULT_SPEEDS, cfg = LOCOMOTION_DEFAULTS) {
  const g = clamp(finite(gait), 0, 1);
  if (backward) return speeds.Walk_Back * (1 + (cfg.backRunFactor - 1) * g);
  const run = speeds.Run_Fwd ?? speeds.Walk_Fwd * 2;
  return speeds.Walk_Fwd + (run - speeds.Walk_Fwd) * g;
}

/**
 * Locomotion state from the actual signed speed (+ forward, - backward).
 * Hysteresis avoids flicker at the thresholds. Returns { state, clip, timeScale }.
 * timeScale = |speed| / natural ground speed of the clip, so the in-place
 * cycle advances exactly as fast as the root travels (no foot skate).
 */
export function selectLocomotion(prevState, signedSpeed, speeds = DEFAULT_SPEEDS, cfg = LOCOMOTION_DEFAULTS, available = null) {
  const a = Math.abs(finite(signedSpeed));
  const has = (name) => !available || available.has(name);
  let state;
  if (prevState === 'Idle' || !prevState) state = a > cfg.idleExit ? 'Walk' : 'Idle';
  else state = a < cfg.idleEnter ? 'Idle' : 'Walk';
  if (state === 'Walk' && signedSpeed > 0) {
    if (prevState === 'Run' ? a > cfg.runExit : a > cfg.runEnter) state = 'Run';
  }
  if (state === 'Idle') return { state, clip: has('Idle') ? 'Idle' : null, timeScale: 1, direction: 0 };
  const backward = signedSpeed < 0;
  let clip = state === 'Run' ? 'Run_Fwd' : backward ? 'Walk_Back' : 'Walk_Fwd';
  // Graceful fallbacks when a clip is missing from the asset.
  if (!has(clip) && clip === 'Run_Fwd') clip = 'Walk_Fwd';
  let reversed = false;
  if (!has(clip) && clip === 'Walk_Back' && has('Walk_Fwd')) { clip = 'Walk_Fwd'; reversed = true; }
  if (!has(clip)) return { state, clip: null, timeScale: 0, direction: backward ? -1 : 1 };
  const natural = (clip === 'Walk_Fwd' && reversed ? speeds.Walk_Back : speeds[clip]) || speeds.Walk_Fwd;
  const timeScale = (reversed ? -1 : 1) * a / natural;
  return { state, clip, timeScale, direction: backward ? -1 : 1 };
}

/**
 * The single movement controller. update(dt, input, forward) integrates with
 * delta time and returns a snapshot. `forward` is the reference ground
 * direction {x, z} (camera view projected on the floor).
 */
export function createMovementController(options = {}) {
  const cfg = { ...LOCOMOTION_DEFAULTS, ...(options.config || {}) };
  let speeds = { ...DEFAULT_SPEEDS, ...(options.speeds || {}) };
  const s = {
    x: 0, z: 0, yaw: finite(options.yaw), speed: 0, targetSpeed: 0,
    backward: false, input: { x: 0, y: 0, gait: 0, source: 'none' }, moveDir: { x: 0, z: 0 },
  };

  function update(dt, input, forward = { x: 0, z: 1 }) {
    const step = Math.max(0, finite(dt));
    const inp = input && (input.x || input.y) ? input : null;
    s.input = inp ? { x: inp.x, y: inp.y, gait: inp.gait || 0, source: inp.source || 'unknown' } : { x: 0, y: 0, gait: 0, source: 'none' };
    let fx = finite(forward.x), fz = finite(forward.z);
    const fl = Math.hypot(fx, fz) || 1; fx /= fl; fz /= fl;
    const rx = -fz, rz = fx; // right of forward on the floor (Y up)
    let target = 0;
    if (inp) {
      const mag = Math.min(1, Math.hypot(inp.x, inp.y));
      const dx = rx * inp.x + fx * inp.y, dz = rz * inp.x + fz * inp.y;
      const dl = Math.hypot(dx, dz);
      const ux = dx / dl, uz = dz / dl;
      s.moveDir = { x: ux, z: uz };
      const angle = Math.acos(clamp(ux * fx + uz * fz, -1, 1)) * 180 / Math.PI;
      s.backward = s.backward ? angle > cfg.backExitDeg : angle > cfg.backEnterDeg;
      // Backward: face away from the movement direction and use Walk_Back.
      const targetYaw = s.backward ? Math.atan2(-ux, -uz) : Math.atan2(ux, uz);
      const err = wrapAngle(targetYaw - s.yaw);
      const turn = clamp(err * (1 - Math.exp(-cfg.turnRate * step)), -cfg.maxTurnSpeed * step, cfg.maxTurnSpeed * step);
      s.yaw = wrapAngle(s.yaw + turn);
      const remaining = Math.abs(wrapAngle(targetYaw - s.yaw));
      // Turn mostly in place before translating when the error is large.
      const align = Math.max(0, Math.cos(remaining));
      target = mag * maxSpeed(s.backward, inp.gait, speeds, cfg) * align * (s.backward ? -1 : 1);
    }
    s.targetSpeed = target;
    // Accelerate / decelerate toward the signed target (through zero on reversal).
    const v = s.speed;
    const speedingUp = Math.sign(target) === Math.sign(v) || v === 0 ? Math.abs(target) > Math.abs(v) : false;
    const rate = speedingUp ? cfg.accel : cfg.decel;
    const dv = clamp(target - v, -rate * step, rate * step);
    s.speed = v + dv;
    if (Math.abs(s.speed) < 1e-4 && target === 0) s.speed = 0;
    s.x = clamp(s.x + Math.sin(s.yaw) * s.speed * step, -cfg.bounds, cfg.bounds);
    s.z = clamp(s.z + Math.cos(s.yaw) * s.speed * step, -cfg.bounds, cfg.bounds);
    return snapshot();
  }

  function snapshot() {
    return {
      position: { x: s.x, y: 0, z: s.z }, yaw: s.yaw, speed: s.speed, targetSpeed: s.targetSpeed,
      backward: s.backward, input: { ...s.input }, moveDir: { ...s.moveDir },
    };
  }

  return {
    update,
    snapshot,
    reset(pos = { x: 0, z: 0 }, yaw = 0) { Object.assign(s, { x: pos.x || 0, z: pos.z || 0, yaw, speed: 0, targetSpeed: 0, backward: false }); },
    setSpeeds(next) { speeds = { ...speeds, ...next }; },
    get speeds() { return { ...speeds }; },
    get config() { return { ...cfg }; },
  };
}
