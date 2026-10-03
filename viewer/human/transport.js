/**
 * Transport. Receives a finished 33-element actuator command and writes the
 * existing runtime WebSocket. It does not read sticks, buttons, or joint names.
 *
 * Contract (one JSON text frame):
 *   {"op":"human_cmd","u":[<33 finite numbers>],"t":<milliseconds>}
 * Sent at HUMAN_CMD_HZ (50). The physics loop consumes the latest vector; this
 * module does not know the MuJoCo timestep.
 */
import { HUMAN_CMD_HZ } from './gamepad-map.js';

export { HUMAN_CMD_HZ };

export function humanCmdMessage(u, t) {
  if (!Array.isArray(u) || u.length !== 33 || u.some((n) => typeof n !== 'number' || !Number.isFinite(n))) {
    throw new TypeError('human_cmd u must be 33 finite numbers');
  }
  if (typeof t !== 'number' || !Number.isFinite(t)) throw new TypeError('human_cmd t must be milliseconds');
  return { op: 'human_cmd', u, t };
}

export function encodeHumanCmd(u, t = Date.now()) {
  return JSON.stringify(humanCmdMessage(u, t));
}

/**
 * getSocket() -> WebSocket | null
 * getU() -> number[33] | null  (null skips the tick, e.g. before inventory)
 */
export function startCommandPump(getSocket, getU, options = {}) {
  const hz = options.hz || HUMAN_CMD_HZ;
  const setTimer = options.setIntervalFn || setInterval;
  const clearTimer = options.clearIntervalFn || clearInterval;
  const now = options.now || Date.now;
  const id = setTimer(() => {
    try {
      const socket = getSocket();
      if (!socket || socket.readyState !== 1) return;
      const u = getU();
      if (!u) return;
      socket.send(encodeHumanCmd(u, now()));
    } catch (err) {
      console.log('human_cmd send failed', err && err.message ? err.message : err);
    }
  }, 1000 / hz);
  return () => clearTimer(id);
}
