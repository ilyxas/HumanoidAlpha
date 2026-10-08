/**
 * Joint mapper. Turns a normalized HumanInput plus the model's actuator
 * inventory into a complete 33-element u_cmd. It does not know whether the
 * stick came from a gamepad or a mouse, and it does not know about WebSockets.
 *
 * Two slots share one cycle of every joint in the inventory. There is no
 * left-body list, no right-body list, and no center-joint owner.
 *   Slot A is the left stick (L1 / the left cycle button).
 *   Slot B is the right stick (R1 / the right cycle button).
 * Either slot may land on any joint, so every actuator in the 33-motor list
 * is reachable from both cyclers. No anatomical side filter.
 *
 * Conflict ban: the same ctrl index is never selected by both slots. Cycling
 * skips a joint that uses any ctrl index the other slot holds and wraps to the
 * next free joint. select() refuses that joint and leaves the slot where it
 * was. command() rebuilds u from zeros and, if a selection ever overlapped,
 * the later slot does not write those channels.
 *
 * 3-axis ball motors (name ending _x_/_y_/_z_motor) keep those real axes.
 * A joint with a single non-XYZ actuator exposes only that actuator on the
 * vertical stick. No synthetic X/Y/Z is invented for a 1-DOF hinge.
 */
import { applyAuthority, applyDeadzone, applyExpo, mapNormalizedToCtrl, rangeContainsZero } from './normalize.js';

export const NU = 33;

/** Stable cycle order. Not a side filter: both slots walk this whole list. */
const PREFERRED_ORDER = [
  'shoulder_l', 'elbow_l', 'wrist_l_flex', 'wrist_l_dev', 'hip_l', 'knee_l', 'ankle_l_dp', 'ankle_l_ie',
  'shoulder_r', 'elbow_r', 'wrist_r_flex', 'wrist_r_dev', 'hip_r', 'knee_r', 'ankle_r_dp', 'ankle_r_ie',
  'lumbar', 'thoracic', 'neck',
];

export const SLOT_BY_STICK = { left: 'A', right: 'B' };

function sortNamed(list, preferred) {
  const rank = new Map(preferred.map((n, i) => [n, i]));
  return [...list].sort((a, b) => (rank.get(a.name) ?? 1000) - (rank.get(b.name) ?? 1000) || a.name.localeCompare(b.name));
}

export function jointCtrlIds(joint) {
  if (joint.kind === 'hinge') return [joint.actuator.id];
  return Object.values(joint.axes).map((a) => a.id);
}

export function buildJointCatalog(actuators) {
  if (!Array.isArray(actuators) || actuators.length !== NU) {
    throw new Error(`expected ${NU} actuators`);
  }
  const groups = new Map();
  for (const a of actuators) {
    if (!a || typeof a.joint !== 'string' || !a.joint) throw new Error('actuator missing joint name');
    if (!Number.isInteger(a.id) || a.id < 0 || a.id >= NU) throw new Error('actuator id out of range');
    if (!Array.isArray(a.ctrlrange) || a.ctrlrange.length !== 2) throw new Error('actuator missing ctrlrange');
    if (!groups.has(a.joint)) groups.set(a.joint, []);
    groups.get(a.joint).push(a);
  }
  const joints = [];
  for (const [name, list] of groups) {
    const axes = {};
    const hinges = [];
    for (const a of list) {
      const entry = { id: a.id, name: a.name, lo: Number(a.ctrlrange[0]), hi: Number(a.ctrlrange[1]) };
      const m = String(a.name).match(/_([xyz])_motor$/);
      if (m) axes[m[1]] = entry;
      else hinges.push(entry);
    }
    if (hinges.length === 0) {
      joints.push({ name, kind: 'xyz', axes });
    } else if (hinges.length === 1 && !axes.x && !axes.y && !axes.z) {
      joints.push({ name, kind: 'hinge', actuator: hinges[0] });
    } else {
      if (axes.x || axes.y || axes.z) joints.push({ name, kind: 'xyz', axes });
      for (const h of hinges) {
        joints.push({ name: h.name.replace(/_motor$/, ''), kind: 'hinge', actuator: h });
      }
    }
  }
  const cycle = sortNamed(joints, PREFERRED_ORDER);
  return {
    joints,
    cycle,
    zeroOutsideRange: joints.filter((j) => {
      const axes = j.kind === 'hinge' ? [j.actuator] : Object.values(j.axes);
      return axes.some((a) => !rangeContainsZero(a.lo, a.hi));
    }).map((j) => j.name),
  };
}

function writeActuator(u, act, norm, authority) {
  const mapped = mapNormalizedToCtrl(norm, act.lo, act.hi);
  u[act.id] = applyAuthority(mapped, authority, act.lo, act.hi);
}

/** Deadzone, then expo. One curve for every axis. Authority is applied later. */
function shapeAxis(raw, gamma) {
  return applyExpo(applyDeadzone(raw), gamma);
}

function writeJoint(u, joint, stick, authority, gamma) {
  const vertical = shapeAxis(stick.vertical, gamma);
  const horizontal = shapeAxis(stick.horizontal, gamma);
  if (joint.kind === 'hinge') {
    writeActuator(u, joint.actuator, vertical, authority);
    return;
  }
  if (joint.axes.x) writeActuator(u, joint.axes.x, vertical, authority);
  const axisName = stick.zModifier ? 'z' : 'y';
  const act = joint.axes[axisName];
  if (act) writeActuator(u, act, horizontal, authority);
  // Z modifier with no Z actuator writes nothing on horizontal, so the Y
  // command is zero while Z is held and stays zero if Z does not exist.
  // Releasing Z simply stops writing Z (that element stays 0). No latch.
}

export function createMapper(actuators) {
  const catalog = buildJointCatalog(actuators);
  const list = catalog.cycle;
  if (list.length < 2) throw new Error('need two joints for slots A and B');

  function overlaps(i, j) {
    if (i === j) return true;
    const ids = new Set(jointCtrlIds(list[i]));
    return jointCtrlIds(list[j]).some((id) => ids.has(id));
  }

  let indexA = list.findIndex((j) => j.name === 'shoulder_l');
  let indexB = list.findIndex((j) => j.name === 'shoulder_r');
  if (indexA < 0) indexA = 0;
  if (indexB < 0) indexB = (indexA + 1) % list.length;
  if (overlaps(indexA, indexB)) {
    const free = list.findIndex((_, i) => !overlaps(indexA, i));
    if (free < 0) throw new Error('no two non-overlapping actuators for slots A and B');
    indexB = free;
  }
  const index = { left: indexA, right: indexB };

  function other(side) {
    return side === 'left' ? 'right' : 'left';
  }

  function selected(side) {
    return list[index[side]];
  }

  /** Advance to the next joint whose ctrl indices are not held by the other slot. Wraps. Stays put only if nothing is free. */
  function cycle(side) {
    const n = list.length;
    const cur = index[side];
    const held = index[other(side)];
    for (let step = 1; step <= n; step++) {
      const i = (cur + step) % n;
      if (!overlaps(i, held)) {
        index[side] = i;
        return list[i].name;
      }
    }
    return list[cur].name;
  }

  /**
   * Move this slot to `name`. Unknown names throw. A joint that shares any
   * ctrl index with the other slot is refused: the index does not change.
   */
  function select(side, name) {
    const i = list.findIndex((j) => j.name === name);
    if (i < 0) throw new Error(`unknown joint ${name}`);
    if (overlaps(i, index[other(side)])) return list[index[side]].name;
    index[side] = i;
    return list[i].name;
  }

  /**
   * input: { left:{horizontal,vertical,zModifier}, right:{...}, authority, expo }
   * expo is gamma (>= 1) applied after the deadzone. Missing expo is linear.
   * Returns a fresh 33-vector. Actuators the slot is not writing are 0, so a
   * joint change clears the previous joint and a centered stick is zero.
   * Each ctrl index is written by at most one slot.
   */
  function command(input) {
    const u = new Array(NU).fill(0);
    const view = {};
    const taken = new Set();
    for (const side of ['left', 'right']) {
      const joint = selected(side);
      const stick = input[side] || { horizontal: 0, vertical: 0, zModifier: false };
      const ids = jointCtrlIds(joint);
      const blocked = ids.some((id) => taken.has(id));
      if (!blocked) {
        writeJoint(u, joint, stick, input.authority, input.expo);
        for (const id of ids) taken.add(id);
      }
      const slot = SLOT_BY_STICK[side];
      view[side] = {
        slot,
        joint: joint.name,
        kind: joint.kind,
        zAvailable: joint.kind === 'xyz' && !!joint.axes.z,
        zActive: !!(stick.zModifier && joint.kind === 'xyz' && joint.axes.z),
        owns: !blocked,
        heldBy: blocked ? SLOT_BY_STICK[other(side)] : slot,
      };
    }
    return { u, view, zeroOutsideRange: catalog.zeroOutsideRange };
  }

  return {
    catalog,
    cycle,
    select,
    selected,
    command,
  };
}
