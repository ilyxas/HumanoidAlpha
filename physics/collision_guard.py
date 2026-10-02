"""Collision-aware, swept Position edits. Never steps or changes physics time."""
import math

import mujoco
import numpy as np


class PositionCollisionGuard:
    # Mesh feet extend 1.61 mm below z=0 in the exported reference pose.
    # Numerical/contact tolerance, not permission to cross another body.
    TOLERANCE = 0.002
    # Match the physical contact margin. Avoid placing limbs inside the contact
    # spring in Position mode and causing a repulsive kick on Resume.
    SELF_CLEARANCE = 0.006
    MAX_ANGLE_STEP = 0.005
    MAX_TRANSLATION_STEP = 0.002

    def __init__(self, model):
        self.model = model
        self.probe = mujoco.MjData(model)

    def depths(self, qpos):
        self.probe.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.probe)
        mujoco.mj_comPos(self.model, self.probe)
        mujoco.mj_collision(self.model, self.probe)
        depths = {}
        for contact in self.probe.contact:
            pair = tuple(sorted((int(contact.geom1), int(contact.geom2))))
            depths[pair] = max(depths.get(pair, -math.inf), -float(contact.dist))
        return depths

    def validate(self, current, target):
        """Reject the entire edit at the first obstruction, including mid-path.

        Existing penetrations after dynamic contact may be relieved, but cannot
        deepen. Probe data is private: a rejected command leaves all live state
        (including velocities, warmstart, contacts and time) untouched.
        """
        m = self.model
        velocity = np.zeros(m.nv)
        mujoco.mj_differentiatePos(m, velocity, 1., current, target)
        # Root's first three dofs are translation; all others are angular.
        steps = max(1, math.ceil(np.linalg.norm(velocity[:3]) / self.MAX_TRANSLATION_STEP),
                    math.ceil(np.linalg.norm(velocity[3:]) / self.MAX_ANGLE_STEP))
        allowed = self.depths(current)
        for step in range(1, steps + 1):
            sample = current.copy()
            mujoco.mj_integratePos(m, sample, velocity, step / steps)
            depths = self.depths(sample)
            for pair, depth in depths.items():
                tolerance = self.TOLERANCE if 0 in pair else -self.SELF_CLEARANCE
                if depth > max(tolerance, allowed.get(pair, -math.inf) + 1e-7):
                    names = ' / '.join(m.geom(i).name for i in pair)
                    raise ValueError(f'Movement blocked by contact: {names}')
                # Permit backing out of a dynamic overlap, but not leaving and
                # re-entering it later in the same swept movement.
                allowed[pair] = min(allowed.get(pair, tolerance), depth)
            for pair in allowed.keys() - depths.keys():
                allowed[pair] = -math.inf
