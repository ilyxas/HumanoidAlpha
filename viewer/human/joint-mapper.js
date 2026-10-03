/**
 * Joint mapper. Turns a normalized HumanInput plus the model's actuator
 * inventory into a complete 33-element u_cmd. It does not know whether the
 * stick came from a gamepad or a mouse, and it does not know about WebSockets.
 *
 * Cycle lists are built from the actuator inventory (joint name, ctrlrange, id).
 * 3-axis ball motors (name ending _x_/_y_/_z_motor) keep those real axes.
 * A joint with a single non-XYZ actuator (knee, elbow, and each ankle/wrist
 * hinge in this model) exposes only that actuator on the vertical stick.
 * No synthetic X/Y/Z is invented for a 1-DOF hinge.
 *
 * Center joints (no _l/_r in the joint name: lumbar, thoracic, neck) appear on
 * both cycle lists. Only the side that currently owns the joint writes it.
 * Cycling onto a center joint claims it and, if the other side was parked on
 * it, moves that side back to its first limb joint. Same-frame calls are
 * deterministic: the caller invokes left then right, so a simultaneous claim
 * resolves to whichever side cycles last (right, when both are cycled together
 * by sample()).
 */
import { applyAuthority, applyDeadzone, mapNormalizedToCtrl, rangeContainsZero } from './normalize.js';

export const NU = 33;

const PREFERRED = {
  left: ['shoulder_l', 'elbow_l', 'wrist_l_flex', 'wrist_l_dev', 'hip_l', 'knee_l', 'ankle_l_dp', 'ankle_l_ie'],
  right: ['shoulder_r', 'elbow_r', 'wrist_r_flex', 'wrist_r_dev', 'hip_r', 'knee_r', 'ankle_r_dp', 'ankle_r_ie'],
  center: ['lumbar', 'thoracic', 'neck'],
};

export function jointSide(name) {
  if (/_l(?:_|$)/.test(name)) return 'left';
  if (/_r(?:_|$)/.test(name)) return 'right';
  return 'center';
}

function sortNamed(list, preferred) {
  const rank = new Map(preferred.map((n, i) => [n, i]));
  return [...list].sort((a, b) => (rank.get(a.name) ?? 1000) - (rank.get(b.name) ?? 1000) || a.name.localeCompare(b.name));
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
    const side = jointSide(name);
    if (hinges.length === 0) {
      joints.push({ name, side, kind: 'xyz', axes });
    } else if (hinges.length === 1 && !axes.x && !axes.y && !axes.z) {
      joints.push({ name, side, kind: 'hinge', actuator: hinges[0] });
    } else {
      if (axes.x || axes.y || axes.z) joints.push({ name, side, kind: 'xyz', axes });
      for (const h of hinges) {
        joints.push({ name: h.name.replace(/_motor$/, ''), side: jointSide(h.name), kind: 'hinge', actuator: h });
      }
    }
  }
  const leftLimbs = sortNamed(joints.filter((j) => j.side === 'left'), PREFERRED.left);
  const rightLimbs = sortNamed(joints.filter((j) => j.side === 'right'), PREFERRED.right);
  const center = sortNamed(joints.filter((j) => j.side === 'center'), PREFERRED.center);
  return {
    joints,
    cycles: {
      left: [...leftLimbs, ...center],
      right: [...rightLimbs, ...center],
    },
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

function writeJoint(u, joint, stick, authority) {
  const vertical = applyDeadzone(stick.vertical);
  const horizontal = applyDeadzone(stick.horizontal);
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
  const index = { left: 0, right: 0 };
  const centerOwner = {};

  function selected(side) {
    const list = catalog.cycles[side];
    if (!list.length) throw new Error(`no joints for ${side}`);
    return list[index[side]];
  }

  function other(side) {
    return side === 'left' ? 'right' : 'left';
  }

  function claim(side) {
    const joint = selected(side);
    if (joint.side !== 'center') return;
    centerOwner[joint.name] = side;
    const o = other(side);
    if (selected(o).name === joint.name) {
      // First entry of the other cycle is its first limb, never a center joint.
      index[o] = 0;
    }
  }

  function owns(side, joint) {
    if (joint.side !== 'center') return joint.side === side;
    return centerOwner[joint.name] === side;
  }

  function cycle(side) {
    const list = catalog.cycles[side];
    index[side] = (index[side] + 1) % list.length;
    claim(side);
    return selected(side).name;
  }

  function select(side, name) {
    const i = catalog.cycles[side].findIndex((j) => j.name === name);
    if (i < 0) throw new Error(`unknown joint ${name} on ${side}`);
    index[side] = i;
    claim(side);
    return selected(side).name;
  }

  /**
   * input: { left:{horizontal,vertical,zModifier}, right:{...}, authority }
   * Returns a fresh 33-vector. Unowned and unselected actuators are 0.
   * Switching joints therefore clears the previous selection. A centered
   * stick deadzones to 0 for the axes that stick owns.
   */
  function command(input) {
    const u = new Array(NU).fill(0);
    const view = {};
    for (const side of ['left', 'right']) {
      const joint = selected(side);
      const stick = input[side] || { horizontal: 0, vertical: 0, zModifier: false };
      const can = owns(side, joint);
      if (can) writeJoint(u, joint, stick, input.authority);
      view[side] = {
        joint: joint.name,
        kind: joint.kind,
        zAvailable: joint.kind === 'xyz' && !!joint.axes.z,
        zActive: !!(stick.zModifier && joint.kind === 'xyz' && joint.axes.z),
        owns: can,
        heldBy: joint.side === 'center' ? (centerOwner[joint.name] || null) : joint.side,
      };
    }
    return { u, view, zeroOutsideRange: catalog.zeroOutsideRange };
  }

  return {
    catalog,
    cycle,
    select,
    selected,
    owns,
    command,
    centerOwner,
  };
}
