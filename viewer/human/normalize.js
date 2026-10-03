/**
 * Device-facing normalization. No MuJoCo, no socket, no actuator names.
 * Output is a normalized stick in [-1, +1]: +vertical is UP (+X in the mapper),
 * +horizontal is RIGHT (+Y, or +Z while the Z modifier is held).
 */
import { DEADZONE, GAMEPAD_MAP } from './gamepad-map.js';

export function applyDeadzone(x, dz = DEADZONE) {
  if (typeof x !== 'number' || !Number.isFinite(x)) return 0;
  const ax = Math.abs(x);
  if (ax <= dz) return 0;
  const denom = 1 - dz;
  if (!(denom > 0)) return 0;
  const scaled = (ax - dz) / denom;
  return Math.sign(x) * Math.min(1, scaled);
}

/**
 * Map a normalized [-1, +1] command into an actuator ctrlrange.
 * If 0 is inside [lo, hi] (the usual symmetric and asymmetric cases):
 *   negative side maps linearly onto [lo, 0], positive side onto [0, hi].
 * If 0 is outside [lo, hi]: map [-1, +1] linearly across [lo, hi].
 * That second case is not used by the current humanoid inventory; it is
 * documented so a future asymmetric motor that excludes 0 does not silently
 * pretend the range is centered.
 */
export function mapNormalizedToCtrl(norm, lo, hi) {
  const n = Math.max(-1, Math.min(1, Number.isFinite(norm) ? norm : 0));
  if (lo <= 0 && hi >= 0) {
    if (n >= 0) return n * hi;
    return (-n) * lo;
  }
  return lo + ((n + 1) / 2) * (hi - lo);
}

export function rangeContainsZero(lo, hi) {
  return lo <= 0 && hi >= 0;
}

/** mapped_ctrl * authority, clipped to the real ctrlrange. */
export function applyAuthority(mapped, authority, lo, hi) {
  const a = Math.max(0, Math.min(1, Number.isFinite(authority) ? authority : 0));
  const v = mapped * a;
  if (!Number.isFinite(v)) return 0;
  return Math.min(hi, Math.max(lo, v));
}

/** Clamp a stick vector to the unit disc. Screen-Y is already flipped by the caller. */
export function clampStick(horizontal, vertical) {
  let h = Number.isFinite(horizontal) ? horizontal : 0;
  let v = Number.isFinite(vertical) ? vertical : 0;
  const m = Math.hypot(h, v);
  if (m > 1) {
    h /= m;
    v /= m;
  }
  return { horizontal: h, vertical: v };
}

/**
 * Read one side's stick from a Gamepad axes array.
 * AXIS_Y_SIGN flips the usual "up = -1" report so UP is +vertical.
 */
export function stickFromGamepadAxes(axes, side, map = GAMEPAD_MAP) {
  const iH = side === 'left' ? map.axes.leftX : map.axes.rightX;
  const iV = side === 'left' ? map.axes.leftY : map.axes.rightY;
  const h = Number(axes?.[iH] ?? 0);
  const vRaw = Number(axes?.[iV] ?? 0);
  return {
    horizontal: Number.isFinite(h) ? h : 0,
    vertical: Number.isFinite(vRaw) ? map.AXIS_Y_SIGN * vRaw : 0,
  };
}

export function buttonDown(button) {
  if (button == null) return false;
  if (typeof button === 'number') return button > 0.5;
  return !!(button.pressed || Number(button.value) > 0.5);
}

/**
 * Never throws. Missing Gamepad API, a thrown getter, or no device all
 * report connected:false.
 */
export function readGamepad(nav) {
  const empty = { connected: false, id: '', mapping: '', axes: [], buttons: [] };
  try {
    const source = nav === undefined && typeof navigator !== 'undefined' ? navigator : nav;
    if (!source || typeof source.getGamepads !== 'function') return empty;
    const pads = source.getGamepads() || [];
    let pad = null;
    for (const candidate of pads) {
      if (candidate && candidate.connected) {
        pad = candidate;
        break;
      }
    }
    if (!pad) return empty;
    const buttons = [];
    const rawButtons = pad.buttons || [];
    for (let i = 0; i < rawButtons.length; i++) {
      const b = rawButtons[i];
      buttons.push({ pressed: !!b?.pressed, value: Number(b?.value) || 0 });
    }
    return {
      connected: true,
      id: pad.id || '',
      mapping: pad.mapping || '',
      axes: Array.from(pad.axes || [], (v) => Number(v) || 0),
      buttons,
    };
  } catch (err) {
    console.log('gamepad read failed', err && err.message ? err.message : err);
    return empty;
  }
}
