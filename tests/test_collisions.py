"""Actual contacts, force response and swept pose edits; no slider-only checks."""
import json
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
from control_console import Controller


class CollisionTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller()

    def test_original_dynamics_properties_and_actuators_preserved(self):
        old = mujoco.MjModel.from_xml_path(str(ROOT / 'reports/collision/model_before.xml'))
        new = self.c.m
        for field in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos',
                      'jnt_type', 'jnt_axis', 'jnt_range', 'jnt_limited', 'dof_damping',
                      'dof_armature', 'actuator_gear', 'actuator_ctrlrange', 'actuator_trnid', 'qpos0'):
            np.testing.assert_allclose(getattr(new, field), getattr(old, field), rtol=1e-10, atol=1e-12)
        self.assertEqual(new.nu, 33)
        self.assertEqual(new.opt.timestep, old.opt.timestep)
        np.testing.assert_array_equal(new.opt.gravity, old.opt.gravity)
        self.assertEqual([new.actuator(i).name for i in range(new.nu)],
                         [old.actuator(i).name for i in range(old.nu)])

    def test_rest_has_no_self_penetration(self):
        c = self.c
        for contact in c.d.contact:
            if contact.geom1 != 0 and contact.geom2 != 0:
                self.assertGreaterEqual(contact.dist, 0)

    def test_both_arms_and_legs_blocked_on_swept_paths_with_clear_endpoints(self):
        # All four endpoints are collision-free. Their paths cross another
        # body, so an endpoint-only collision validator would fail this test.
        cases = {
            'shoulder_l': [.3706759936538848, .7842280051635765, .4296142128005694, -.25103261104009916],
            'shoulder_r': [.3945000039667083, -.880918173022919, .02075592141365919, .2606187081437167],
            'hip_l': [.5740540108594414, -.23997895910041603, .3242569620865087, .7125514117196722],
            'hip_r': [.6182582746319311, -.4703867990828236, -.2924425034158958, -.5576471530399907],
        }
        for name, q in cases.items():
            with self.subTest(name=name):
                c = self.c
                c.handle({'op': 'reset'})
                root = c.d.qpos[:7].copy(); root[2] += .6
                c.handle({'op': 'joint', 'id': 0, 'value': root})
                joint = c.m.joint(name)
                target = c.d.qpos.copy(); a = joint.qposadr[0]; target[a:a+4] = q
                self.assertLessEqual(max(c.position_guard.depths(target).values(), default=0), .002)
                before = c.d.qpos.copy(), c.d.qvel.copy(), c.d.qacc_warmstart.copy(), c.d.time, c.revision
                with self.assertRaisesRegex(ValueError, 'blocked by contact'):
                    c.handle({'op': 'joint', 'id': joint.id, 'value': q})
                for actual, expected in zip((c.d.qpos, c.d.qvel, c.d.qacc_warmstart), before[:3]):
                    np.testing.assert_array_equal(actual, expected)
                self.assertEqual((c.d.time, c.revision), before[3:])

    def test_floor_cannot_be_teleported_through(self):
        target = self.c.d.qpos[:7].copy(); target[2] -= 2
        before = self.c.d.qpos.copy()
        with self.assertRaisesRegex(ValueError, 'floor'):
            self.c.handle({'op': 'joint', 'id': 0, 'value': target})
        np.testing.assert_array_equal(self.c.d.qpos, before)

    def test_contact_boundary_allows_moving_back_out(self):
        c = self.c
        j = c.m.joint('shoulder_l')
        accepted = 0
        for angle in np.arange(.02, 1., .02):
            q = [math.cos(angle / 2), 0, math.sin(angle / 2), 0]
            try:
                c.handle({'op': 'joint', 'id': j.id, 'value': q})
                accepted = angle
            except ValueError:
                break
        self.assertGreater(accepted, .1)
        self.assertLess(accepted, .5)
        c.handle({'op': 'joint', 'id': j.id, 'value': [1, 0, 0, 0]})
        np.testing.assert_allclose(c.d.qpos, c.m.qpos0)

    def test_position_to_dynamic_does_not_preload_self_contact(self):
        c = self.c
        root = c.d.qpos[:7].copy(); root[2] += .3
        c.handle({'op': 'joint', 'id': 0, 'value': root})
        joint = c.m.joint('shoulder_l')
        for angle in np.arange(.01, .5, .01):
            try:
                c.handle({'op': 'joint', 'id': joint.id,
                          'value': [math.cos(angle/2), 0, math.sin(angle/2), 0]})
            except ValueError:
                break
        # Isolate contact preload from gravity and commanded motor acceleration.
        c.m.opt.gravity[:] = 0
        # The original model has soft joint-limit margins at its elbow rest
        # position. Compare identical states with/without contacts to separate
        # that existing joint-limit response from collision preload.
        control_model = mujoco.MjModel.from_xml_path(str(ROOT / 'physics/model/humanoid_alpha.xml'))
        control_model.opt.gravity[:] = 0
        control_model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        control_data = mujoco.MjData(control_model)
        control_data.qpos[:] = c.d.qpos
        mujoco.mj_step(control_model, control_data)
        c.handle({'op': 'mode', 'value': 'dynamic'})
        c.handle({'op': 'resume'})
        c.tick()
        np.testing.assert_allclose(c.d.qvel, control_data.qvel, atol=1e-10)

    def test_dynamic_contact_forces_and_stability(self):
        m, d = self.c.m, self.c.d
        reports = []
        all_pairs = set()
        for label, scale, seed in [('gravity', 0, 0), ('quarter_torque', .25, 12),
                                   ('full_torque', 1, 12), ('full_torque_2', 1, 41),
                                   ('full_torque_3', 1, 99),
                                   ('all_positive_limits', 1, None)]:
            mujoco.mj_resetData(m, d)
            rng = np.random.default_rng(seed)
            peak = 0.; peak_self = 0.; positive_forces = 0; deep_run = 0; longest_deep_run = 0
            pairs = set()
            for step in range(2500):
                if step % 100 == 0:
                    d.ctrl[:] = m.actuator_ctrlrange[:, 1] * scale * (1 if seed is None else rng.uniform(-1, 1, m.nu))
                mujoco.mj_step(m, d)
                self.assertTrue(np.isfinite(d.qpos).all())
                depth = 0.
                for i, contact in enumerate(d.contact):
                    peak = max(peak, -contact.dist)
                    if contact.geom1 == 0 or contact.geom2 == 0:
                        continue
                    depth = max(depth, -contact.dist)
                    names = tuple(sorted((m.geom(contact.geom1).name, m.geom(contact.geom2).name)))
                    force = np.zeros(6); mujoco.mj_contactForce(m, d, i, force)
                    if force[0] > .1:
                        pairs.add(names); positive_forces += 1
                peak_self = max(peak_self, depth)
                deep_run = deep_run + 1 if depth > .01 else 0
                longest_deep_run = max(longest_deep_run, deep_run)
            self.assertFalse(d.warning.number.any(), label)
            self.assertGreater(positive_forces, 0, label)
            # Detect gross passage / sustained deep intersection, while recording
            # transient soft-contact errors honestly at maximum motor inputs.
            self.assertLess(peak_self, .015, label)
            self.assertLess(longest_deep_run * m.opt.timestep, .02, label)
            all_pairs.update(pairs)
            reports.append(dict(scenario=label, simulated_seconds=d.time,
                                max_penetration_m=peak, max_self_penetration_m=peak_self,
                                longest_self_overlap_over_10mm_seconds=longest_deep_run*m.opt.timestep,
                                self_contacts_with_force=positive_forces, force_pairs=sorted(pairs),
                                warnings=d.warning.number.tolist()))
        for region in ('upperarm_l', 'upperarm_r', 'lowerarm_l', 'lowerarm_r',
                       'hand_l', 'hand_r', 'thigh_l', 'thigh_r', 'calf_l', 'calf_r', 'foot_l', 'foot_r'):
            self.assertTrue(any(any(name.startswith(region) for name in pair) for pair in all_pairs), region)
        (ROOT / 'reports/collision/dynamic_contacts.json').write_text(json.dumps(reports, indent=2) + '\n')

    def test_motor_driven_arm_is_resisted_by_contact(self):
        depths = []
        for contacts_enabled in (True, False):
            # Diagnostic control experiment: disable contacts only in this
            # private model, to establish that contact forces cause resistance.
            m = mujoco.MjModel.from_xml_path(str(ROOT / 'physics/model/humanoid_alpha.xml'))
            m.opt.gravity[:] = 0
            if not contacts_enabled:
                m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
            d = mujoco.MjData(m); d.qpos[2] += .3
            d.ctrl[m.actuator('shoulder_l_y_motor').id] = 5
            arm, torso = m.geom('upperarm_l_joint_guard').id, m.geom('torso_upper_geom').id
            peak = 0.; force_samples = 0
            for _ in range(250):
                mujoco.mj_step(m, d)
                for i, contact in enumerate(d.contact):
                    if {int(contact.geom1), int(contact.geom2)} == {arm, torso}:
                        force = np.zeros(6); mujoco.mj_contactForce(m, d, i, force)
                        force_samples += force[0] > .1
                mujoco.mj_forward(m, d)
                peak = max(peak, -mujoco.mj_geomDistance(m, d, arm, torso, 1., None))
            depths.append(peak)
            if contacts_enabled:
                self.assertGreater(force_samples, 0)
        self.assertLess(depths[0], .005)
        self.assertGreater(depths[1], .05)
        (ROOT / 'reports/collision/controlled_motor_contact.json').write_text(json.dumps(
            dict(torque_Nm=5, duration_seconds=.5, contact_on_peak_m=depths[0],
                 contact_off_peak_m=depths[1]), indent=2) + '\n')


if __name__ == '__main__':
    unittest.main()
