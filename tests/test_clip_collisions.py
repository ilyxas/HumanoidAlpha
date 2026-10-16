"""physics/clip_collisions.py: collision shapes placed on the clip's bones, mj_collision only."""
import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
sys.path.insert(0, str(ROOT / 'tests'))
from clip_collisions import ClipCollisions, contact_points, events  # noqa: E402
from test_animation_mode import run_node  # noqa: E402


class ClipCollisionsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cc = ClipCollisions()

    def test_contact_points_are_on_each_surface(self):
        xml = ('<mujoco><worldbody><body><freejoint/><geom type="sphere" size="0.1"/></body>'
               '<body pos="0.15 0 0"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>')
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        pa, pb = contact_points(d.contact[0])
        np.testing.assert_allclose(pa, [0.1, 0, 0], atol=1e-9)   # sphere A's surface, inside B
        np.testing.assert_allclose(pb, [0.05, 0, 0], atol=1e-9)  # sphere B's surface, inside A

    def test_bind_pose_matches_kinematics_and_is_clean(self):
        cc = self.cc
        d2 = mujoco.MjData(cc.m)
        d2.qpos[:] = cc.rt.qpos_from(*cc.rt.body_quats(None, 0.0), clamp=False)
        mujoco.mj_kinematics(cc.m, d2)
        cc.place(None, 0.0)
        np.testing.assert_allclose(cc.d.geom_xpos, d2.geom_xpos, atol=1e-6)
        hits = cc.hits()
        # Only the exported feet's 1.6 mm below the floor (docs/COLLISIONS.md).
        self.assertEqual({(h['a'], h['b']) for h in hits}, {('foot_l', 'floor'), ('foot_r', 'floor')})
        self.assertTrue(all(h['depth_mm'] < 2.0 for h in hits))

    def test_jump_hands_touch_thighs_in_the_crouch(self):
        c = self.cc.clip('Jump', fps=10)
        pairs = {(e['a'], e['b']) for e in c['events']}
        self.assertIn(('hand_l', 'thigh_l'), pairs)
        for e in c['events']:
            self.assertEqual(len(e['worst']['pa']), 3)
            self.assertEqual(len(e['worst']['pb']), 3)

    def test_events_merge_consecutive_frames(self):
        hit = dict(a='hand_l', b='hand_r', kind='self', pa=[0, 0, 0], pb=[0, 0, 0.01], depth_mm=10.0)
        frames = [dict(t=0.0, hits=[hit]), dict(t=0.1, hits=[{**hit, 'depth_mm': 12.0}]), dict(t=0.2, hits=[]),
                  dict(t=0.3, hits=[hit])]
        ev = events(frames)
        self.assertEqual([(e['start'], e['end'], e['frames']) for e in ev], [(0.0, 0.1, 2), (0.3, 0.3, 1)])
        self.assertEqual(ev[0]['worst']['depth_mm'], 12.0)

    def test_overlay_helpers_and_report(self):
        result = run_node('test_collision_overlay.mjs')
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        self.assertIn('test_collision_overlay OK', result.stdout)


if __name__ == '__main__':
    unittest.main()
