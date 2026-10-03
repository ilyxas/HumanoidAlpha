/**
 * On-screen sticks. They are both the mouse/touch input and the visualizer
 * of whichever device currently owns that stick. The left stick is slot A and
 * the right stick is slot B; the label is the slot plus the selected joint.
 * No actuator table, no torque readout, no axis dump.
 */
import { DEFAULT_EXPO_GAMMA, expoLabel } from './gamepad-map.js';
import { clampStick } from './normalize.js';

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text != null) node.textContent = text;
  return node;
}

const SLOT = { left: 'A', right: 'B' };

function stickWidget(side, session) {
  const slot = SLOT[side];
  const root = el('section', 'human-stick');
  root.dataset.side = side;
  root.dataset.slot = slot;
  const name = el('div', 'human-joint', '…');
  const legend = el('div', 'human-legend', 'vertical +X up / -X down\nhorizontal -Y left / +Y right');
  const pad = el('div', 'human-pad');
  pad.setAttribute('role', 'slider');
  pad.setAttribute('aria-label', `slot ${slot} stick`);
  pad.tabIndex = 0;
  const ring = el('div', 'human-ring');
  const dot = el('div', 'human-dot');
  const labUp = el('span', 'human-lab up', '+X');
  const labDown = el('span', 'human-lab down', '-X');
  const labLeft = el('span', 'human-lab left', '-Y');
  const labRight = el('span', 'human-lab right', '+Y');
  ring.append(dot, labUp, labDown, labLeft, labRight);
  pad.append(ring);
  const zFlag = el('div', 'human-z', 'Z off');
  const row = el('div', 'human-row');
  const cycle = el('button', 'human-btn', 'cycle');
  cycle.type = 'button';
  cycle.setAttribute('aria-label', `Cycle slot ${slot}`);
  const zBtn = el('button', 'human-btn', 'Z');
  zBtn.type = 'button';
  zBtn.setAttribute('aria-label', `Hold Z modifier slot ${slot}`);
  row.append(cycle, zBtn);
  root.append(name, legend, pad, zFlag, row);

  function fromEvent(event) {
    const rect = ring.getBoundingClientRect();
    const radius = rect.width / 2;
    if (!(radius > 0)) return { horizontal: 0, vertical: 0 };
    const h = (event.clientX - (rect.left + radius)) / radius;
    const v = -((event.clientY - (rect.top + radius)) / radius);
    return clampStick(h, v);
  }

  pad.addEventListener('pointerdown', (event) => {
    pad.setPointerCapture(event.pointerId);
    const s = fromEvent(event);
    session.mouseDown(side, s.horizontal, s.vertical);
    event.preventDefault();
    event.stopPropagation();
  });
  pad.addEventListener('pointermove', (event) => {
    if (!pad.hasPointerCapture(event.pointerId)) return;
    const s = fromEvent(event);
    session.mouseMove(side, s.horizontal, s.vertical);
    event.preventDefault();
  });
  const release = (event) => {
    if (pad.hasPointerCapture(event.pointerId)) pad.releasePointerCapture(event.pointerId);
    session.mouseUp(side);
  };
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);

  cycle.addEventListener('click', (event) => {
    event.stopPropagation();
    session.cycle(side);
  });

  // Hold, matching L2/R2. Releasing the button clears Z immediately (no latch).
  const zDown = (event) => {
    zBtn.setPointerCapture(event.pointerId);
    session.mouseZ(side, true);
    event.preventDefault();
    event.stopPropagation();
  };
  const zUp = (event) => {
    if (zBtn.hasPointerCapture(event.pointerId)) zBtn.releasePointerCapture(event.pointerId);
    session.mouseZ(side, false);
  };
  zBtn.addEventListener('pointerdown', zDown);
  zBtn.addEventListener('pointerup', zUp);
  zBtn.addEventListener('pointercancel', zUp);

  function paint(sample) {
    const view = sample.view[side];
    const stick = sample.sticks[side];
    const label = `${view.slot || slot} · ${view.joint}`;
    name.textContent = view.owns ? label : `${label} · held by slot ${view.heldBy}`;
    const h = stick.horizontal;
    const v = stick.vertical;
    const travel = ring.clientWidth > 0 ? ring.clientWidth / 2 - 10 : 46;
    dot.style.transform = `translate(${h * travel}px, ${-v * travel}px)`;
    if (!view.zAvailable) {
      zFlag.textContent = '1-DOF · vertical only · Z n/a';
      zFlag.dataset.on = '0';
      zBtn.disabled = true;
    } else if (view.zActive) {
      zFlag.textContent = 'Z ON · horizontal → Z';
      zFlag.dataset.on = '1';
      zBtn.disabled = false;
    } else {
      zFlag.textContent = 'Z off · horizontal → Y';
      zFlag.dataset.on = '0';
      zBtn.disabled = false;
    }
    root.dataset.owner = stick.owner;
    root.dataset.owns = view.owns ? '1' : '0';
  }

  return { root, paint };
}

export function mountSticks(parent, session) {
  const bar = el('div', 'human-status');
  const padStatus = el('div', 'human-pad-status', 'GAMEPAD: NOT CONNECTED');
  const controls = el('div', 'human-controls');
  const expoBox = el('div', 'human-expo');
  const expoDown = el('button', 'human-btn', '−');
  expoDown.type = 'button';
  expoDown.setAttribute('aria-label', 'Decrease expo');
  const expoText = el('span', 'human-authority-label', expoLabel(DEFAULT_EXPO_GAMMA));
  const expoUp = el('button', 'human-btn', '+');
  expoUp.type = 'button';
  expoUp.setAttribute('aria-label', 'Increase expo');
  expoBox.append(expoDown, expoText, expoUp);
  const authority = el('div', 'human-authority');
  const down = el('button', 'human-btn', '−');
  down.type = 'button';
  down.setAttribute('aria-label', 'Decrease authority');
  const label = el('span', 'human-authority-label', 'AUTHORITY 10%');
  const up = el('button', 'human-btn', '+');
  up.type = 'button';
  up.setAttribute('aria-label', 'Increase authority');
  authority.append(down, label, up);
  controls.append(expoBox, authority);
  bar.append(padStatus, controls);

  const left = stickWidget('left', session);
  const right = stickWidget('right', session);
  parent.append(left.root, right.root, bar);

  down.addEventListener('click', () => session.stepAuthority(-1));
  up.addEventListener('click', () => session.stepAuthority(1));
  expoDown.addEventListener('click', () => session.stepExpo(-1));
  expoUp.addEventListener('click', () => session.stepExpo(1));

  let logged = false;
  let wasConnected = false;
  function paint(sample) {
    left.paint(sample);
    right.paint(sample);
    const pct = Math.round(sample.authority * 100);
    label.textContent = `AUTHORITY ${pct}%`;
    expoText.textContent = expoLabel(sample.expo);
    padStatus.textContent = sample.gamepadConnected ? 'GAMEPAD: CONNECTED' : 'GAMEPAD: NOT CONNECTED';
    padStatus.title = sample.gamepadConnected ? (sample.gamepadName || 'gamepad') : '';
    if (sample.gamepadConnected !== wasConnected) {
      console.log(sample.gamepadConnected ? `gamepad connected ${sample.gamepadName} mapping=${sample.gamepadMapping || 'unknown'}` : 'gamepad disconnected');
      wasConnected = sample.gamepadConnected;
      logged = sample.gamepadConnected;
    }
  }
  return { paint, logged: () => logged };
}
