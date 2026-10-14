/**
 * Wires the observation page: gamepad + mouse -> session -> 33-vector -> transport.
 * Gamepad reading never touches the socket. The mapper never sees the socket.
 */
import { createSession } from './session.js';
import { readGamepad } from './normalize.js';
import { startCommandPump } from './transport.js';
import { mountSticks } from './sticks-ui.js';

/** options.nu: actuator count of the model (33 default; Xandra-Lite uses 11). */
export function createHumanControls(doc = document, { nu = 33, initial } = {}) {
  const host = doc.getElementById('human');
  let session = null;
  let ui = null;
  let socket = null;
  let stopPump = null;
  let raf = 0;
  let started = false;

  function currentU() {
    if (!session) return null;
    return session.sample().u;
  }

  function loop() {
    raf = requestAnimationFrame(loop);
    if (!session) return;
    session.setGamepad(readGamepad());
    ui.paint(session.sample());
  }

  function ensureRunning() {
    if (started || !session) return;
    started = true;
    ui = mountSticks(host, session);
    stopPump = startCommandPump(() => socket, currentU, { nu });
    raf = requestAnimationFrame(loop);
  }

  return {
    bindSocket(next) { socket = next; },
    onInventory(inventory) {
      const actuators = inventory && inventory.actuators;
      if (!Array.isArray(actuators) || actuators.length !== nu) {
        console.log(`human control waiting for ${nu}-actuator inventory`);
        return;
      }
      if (session) return;
      session = createSession(actuators.map((a) => ({
        id: a.id,
        name: a.name,
        joint: a.joint,
        ctrlrange: a.ctrlrange,
      })), initial ? { nu, initial } : { nu });
      ensureRunning();
    },
    stop() {
      if (raf) cancelAnimationFrame(raf);
      raf = 0;
      if (stopPump) stopPump();
      stopPump = null;
      started = false;
    },
  };
}
