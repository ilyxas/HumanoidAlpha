/**
 * Arbitration and UI-independent session state.
 * One owner per side:
 *   1. While the mouse/touch stick is dragging, mouse owns that side.
 *   2. Else if that side's gamepad stick is outside the deadzone OR L1/R1/L2/R2
 *      for that side is active, gamepad owns that side.
 *   3. Else keep the last owner. A centered stick still means zero command
 *      (the mapper deadzones it). Torque is not latched.
 * Gamepad button indices are read from GAMEPAD_MAP only.
 */
import { AUTHORITY_LEVELS, DEADZONE, DEFAULT_AUTHORITY, GAMEPAD_MAP } from './gamepad-map.js';
import { createMapper } from './joint-mapper.js';
import { buttonDown, stickFromGamepadAxes } from './normalize.js';

export function createSession(actuators) {
  const mapper = createMapper(actuators);
  const mouse = {
    left: { dragging: false, horizontal: 0, vertical: 0, z: false },
    right: { dragging: false, horizontal: 0, vertical: 0, z: false },
  };
  const lastOwner = { left: 'mouse', right: 'mouse' };
  let gamepad = { connected: false, id: '', mapping: '', axes: [], buttons: [] };
  let prevButtons = [];
  let authorityIndex = Math.max(0, AUTHORITY_LEVELS.indexOf(DEFAULT_AUTHORITY));

  function setGamepad(next) {
    if (!next || !next.connected) {
      gamepad = { connected: false, id: '', mapping: '', axes: [], buttons: [] };
      return;
    }
    gamepad = next;
  }

  function triggerIndex(side) {
    return side === 'left' ? GAMEPAD_MAP.buttons.L2 : GAMEPAD_MAP.buttons.R2;
  }

  function sideButtonsActive(side) {
    if (!gamepad.connected) return false;
    const ids = side === 'left'
      ? [GAMEPAD_MAP.buttons.L1, GAMEPAD_MAP.buttons.L2]
      : [GAMEPAD_MAP.buttons.R1, GAMEPAD_MAP.buttons.R2];
    return ids.some((i) => buttonDown(gamepad.buttons[i]));
  }

  function rose(i) {
    return buttonDown(gamepad.buttons[i]) && !buttonDown(prevButtons[i]);
  }

  function stepAuthority(dir) {
    const next = authorityIndex + dir;
    if (next < 0 || next >= AUTHORITY_LEVELS.length) return AUTHORITY_LEVELS[authorityIndex];
    authorityIndex = next;
    return AUTHORITY_LEVELS[authorityIndex];
  }

  function applyGamepadEdges() {
    if (!gamepad.connected) {
      prevButtons = [];
      return;
    }
    if (rose(GAMEPAD_MAP.buttons.L1)) mapper.cycle('left');
    if (rose(GAMEPAD_MAP.buttons.R1)) mapper.cycle('right');
    if (rose(GAMEPAD_MAP.buttons.DpadUp)) stepAuthority(1);
    if (rose(GAMEPAD_MAP.buttons.DpadDown)) stepAuthority(-1);
    prevButtons = gamepad.buttons.map((b) => ({ pressed: !!b?.pressed, value: Number(b?.value) || 0 }));
  }

  function resolve(side) {
    const m = mouse[side];
    const stick = stickFromGamepadAxes(gamepad.axes, side);
    const outside = gamepad.connected && (Math.abs(stick.horizontal) > DEADZONE || Math.abs(stick.vertical) > DEADZONE);
    const buttons = sideButtonsActive(side);
    let owner = lastOwner[side];
    let horizontal = 0;
    let vertical = 0;
    let zModifier = false;
    if (m.dragging) {
      owner = 'mouse';
      horizontal = m.horizontal;
      vertical = m.vertical;
      zModifier = m.z;
    } else if (outside || buttons) {
      owner = 'gamepad';
      horizontal = stick.horizontal;
      vertical = stick.vertical;
      zModifier = buttonDown(gamepad.buttons[triggerIndex(side)]);
    } else if (owner === 'gamepad') {
      horizontal = stick.horizontal;
      vertical = stick.vertical;
      zModifier = gamepad.connected && buttonDown(gamepad.buttons[triggerIndex(side)]);
    } else {
      owner = 'mouse';
      horizontal = m.horizontal;
      vertical = m.vertical;
      zModifier = m.z;
    }
    lastOwner[side] = owner;
    return { horizontal, vertical, zModifier, owner };
  }

  function sample() {
    applyGamepadEdges();
    const left = resolve('left');
    const right = resolve('right');
    const authority = AUTHORITY_LEVELS[authorityIndex];
    const built = mapper.command({ left, right, authority });
    return {
      u: built.u,
      view: built.view,
      zeroOutsideRange: built.zeroOutsideRange,
      authority,
      sticks: {
        left: { horizontal: left.horizontal, vertical: left.vertical, zModifier: left.zModifier, owner: left.owner },
        right: { horizontal: right.horizontal, vertical: right.vertical, zModifier: right.zModifier, owner: right.owner },
      },
      gamepadConnected: !!gamepad.connected,
      gamepadName: gamepad.id || '',
      gamepadMapping: gamepad.mapping || '',
    };
  }

  return {
    mapper,
    sample,
    setGamepad,
    stepAuthority,
    cycle(side) { return mapper.cycle(side); },
    select(side, name) { return mapper.select(side, name); },
    mouseDown(side, horizontal, vertical) {
      const m = mouse[side];
      m.dragging = true;
      m.horizontal = horizontal;
      m.vertical = vertical;
    },
    mouseMove(side, horizontal, vertical) {
      const m = mouse[side];
      if (!m.dragging) return;
      m.horizontal = horizontal;
      m.vertical = vertical;
    },
    mouseUp(side) {
      const m = mouse[side];
      m.dragging = false;
      m.horizontal = 0;
      m.vertical = 0;
    },
    /** Hold, not a latch. pointerup must call this with false so Z goes to zero. */
    mouseZ(side, held) {
      mouse[side].z = !!held;
    },
    get authority() { return AUTHORITY_LEVELS[authorityIndex]; },
  };
}
