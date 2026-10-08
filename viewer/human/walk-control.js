/**
 * Animation mode (no physics). A stick session with the same surface the
 * on-screen sticks (sticks-ui.js) expect, but it feeds the locomotion
 * controller (locomotion.js) instead of a 33-actuator vector. Nothing here
 * touches a socket or MuJoCo.
 *
 * LEFT stick (EXP002.1-B): locomotion. Direction = movement direction,
 *   magnitude = speed (radial deadzone + the EXPO curve); the outer 20 % of
 *   travel blends walk -> run. -> sample().move (a MoveInput or null).
 * RIGHT stick: kept from Animation mode (4), vertical axis only
 *   (+X on the legend, UP positive for mouse/touch and gamepad):
 *   vertical <= DEADZONE (centered or pulled down) -> nothing
 *   vertical in (DEADZONE, 1] -> walk forward, drive = expo(rescaled)
 *   -> sample().walk; it goes through the same movement controller.
 */
import { DEADZONE, EXPO_GAMMAS, GAMEPAD_MAP } from './gamepad-map.js';
import { applyDeadzone, applyExpo, buttonDown, stickFromGamepadAxes } from './normalize.js';
import { stickInput } from './locomotion.js';

export const WALK_MIN_SPEED = 0.2;
export const WALK_MAX_SPEED = 2.0;
/** Linear by default: the higher the stick, the faster, proportionally. */
export const WALK_DEFAULT_EXPO = 1;

/**
 * Stick vertical value [-1, 1] (+ is UP) -> { active, drive, timeScale }.
 * Pulling down or staying inside the deadzone gives active:false.
 */
export function walkFromStick(vertical, gamma = WALK_DEFAULT_EXPO, dz = DEADZONE) {
  const v = Number.isFinite(vertical) ? vertical : 0;
  if (v <= dz) return { active: false, drive: 0, timeScale: 0 };
  const drive = applyExpo(applyDeadzone(v, dz), gamma);
  const timeScale = WALK_MIN_SPEED + (WALK_MAX_SPEED - WALK_MIN_SPEED) * drive;
  return { active: true, drive, timeScale };
}

export function createWalkSession() {
  const mouse = {
    left: { dragging: false, horizontal: 0, vertical: 0 },
    right: { dragging: false, horizontal: 0, vertical: 0 },
  };
  const lastOwner = { left: 'mouse', right: 'mouse' };
  let gamepad = { connected: false, id: '', mapping: '', axes: [], buttons: [] };
  let expoIndex = Math.max(0, EXPO_GAMMAS.indexOf(WALK_DEFAULT_EXPO));
  let walkLabel = 'walk';
  let moveLabel = 'move';
  let pointerType = 'touch';

  function resolve(side) {
    const m = mouse[side];
    const stick = stickFromGamepadAxes(gamepad.axes, side);
    const ids = side === 'left' ? [GAMEPAD_MAP.buttons.L1, GAMEPAD_MAP.buttons.L2] : [GAMEPAD_MAP.buttons.R1, GAMEPAD_MAP.buttons.R2];
    const outside = gamepad.connected && (Math.abs(stick.horizontal) > DEADZONE || Math.abs(stick.vertical) > DEADZONE
      || ids.some((i) => buttonDown(gamepad.buttons[i])));
    let owner = lastOwner[side];
    let s;
    if (m.dragging) { owner = 'mouse'; s = m; }
    else if (outside || (owner === 'gamepad' && gamepad.connected)) { owner = 'gamepad'; s = stick; }
    else { owner = 'mouse'; s = m; }
    lastOwner[side] = owner;
    return { horizontal: s.horizontal, vertical: s.vertical, zModifier: false, owner };
  }

  function sample() {
    const left = resolve('left');
    const right = resolve('right');
    const expo = EXPO_GAMMAS[expoIndex];
    const walk = walkFromStick(right.vertical, expo);
    const src = (owner) => (owner === 'gamepad' ? 'gamepad' : pointerType);
    const move = stickInput(left.horizontal, left.vertical, { gamma: expo, source: src(left.owner) });
    const view = {
      left: { slot: 'A', joint: move ? `${moveLabel} ${Math.round(Math.hypot(move.x, move.y) * 100)}%${move.gait > 0 ? ' · run' : ''}` : `${moveLabel} · WASD`, owns: true, heldBy: null, zAvailable: false, zActive: false },
      right: {
        slot: 'B',
        joint: walk.active ? `${walkLabel} ${Math.round(walk.drive * 100)}%` : `${walkLabel} · push up`,
        owns: true, heldBy: null, zAvailable: false, zActive: false,
      },
    };
    return {
      u: null,
      view,
      walk,
      move,
      walkSource: src(right.owner),
      authority: 1,
      expo,
      sticks: { left, right },
      gamepadConnected: !!gamepad.connected,
      gamepadName: gamepad.id || '',
      gamepadMapping: gamepad.mapping || '',
    };
  }

  return {
    sample,
    setGamepad(next) {
      gamepad = next && next.connected ? next : { connected: false, id: '', mapping: '', axes: [], buttons: [] };
    },
    setWalkLabel(name) { walkLabel = String(name || 'walk'); },
    setMoveLabel(name) { moveLabel = String(name || 'move'); },
    /** 'touch' | 'mouse' | 'pen' from the last pointerdown on a stick (input-source readout). */
    setPointerType(type) { pointerType = type === 'mouse' || type === 'pen' ? type : 'touch'; },
    stepAuthority() { return 1; },
    stepExpo(dir) {
      const next = expoIndex + dir;
      if (next >= 0 && next < EXPO_GAMMAS.length) expoIndex = next;
      return EXPO_GAMMAS[expoIndex];
    },
    cycle() { return null; },
    mouseDown(side, horizontal, vertical) { Object.assign(mouse[side], { dragging: true, horizontal, vertical }); },
    mouseMove(side, horizontal, vertical) {
      const m = mouse[side];
      if (!m.dragging) return;
      m.horizontal = horizontal; m.vertical = vertical;
    },
    mouseUp(side) { Object.assign(mouse[side], { dragging: false, horizontal: 0, vertical: 0 }); },
    mouseZ() {},
    get expo() { return EXPO_GAMMAS[expoIndex]; },
  };
}
