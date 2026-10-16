"""physics/clip_react.py: clip-driven body, MuJoCo responds only to self-collisions."""
import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
from clip_collisions import ClipCollisions  # noqa: E402
from clip_react import Reactor  # noqa: E402


class ClipReactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cc = ClipCollisions()
        cls.r = Reactor(cls.cc)

    def test_settings_no_floor_no_limits_no_gravity(self):
        m = self.r.m
        self.assertFalse(any(m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE for g in range(m.ngeom)))
        self.assertFalse(m.jnt_limited.any())
        np.testing.assert_allclose(m.opt.gravity, 0)
        self.assertEqual(m.body_mocapid[m.body('pelvis').id], 0)  # kinematic anchor

    def test_bind_pose_matches_the_collision_audit(self):
        r, cc = self.r, self.cc
        r.reset(None, 0.0)
        cc.place(None, 0.0)
        for name in ('hand_l', 'foot_r', 'head', 'torso_upper'):
            bid, src = r.m.body(name).id, cc.m.body(name).id
            np.testing.assert_allclose(r.d.xpos[bid], cc.bone_heads(None, 0.0)[cc.rt.body_to_bone[name]], atol=1e-5)
            np.testing.assert_allclose(r.d.xquat[bid], cc.d.xquat[src], atol=1e-6)
        self.assertEqual(r.d.ncon, 0)

    def test_without_contacts_the_drives_follow_the_clip(self):
        r = self.r
        r.m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        try:
            c = r.clip('Walk_Fwd', fps=10, baseline=False)
        finally:
            r.m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.assertLess(c['max_dev_mm'], 10.0)

    def test_contacts_push_overlapping_parts_apart(self):
        # Jump's crouch drives the hands into the thighs (clip_collisions: ~46 mm); REACT keeps them
        # within the contact margin and moves the hands away from the clip instead.
        c = self.r.clip('Jump', fps=10, baseline=False)
        deepest = max((h['depth_mm'] for f in c['frames'][1:] for h in f['hits'] if {h['a'], h['b']} == {'hand_l', 'thigh_l'}), default=0)
        self.assertLess(deepest, 15.0)
        self.assertGreater(c['worst_dev']['hand_l'][0], 10.0)


if __name__ == '__main__':
    unittest.main()
