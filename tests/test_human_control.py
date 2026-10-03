"""Human command path: harness ctrl, stale timeout, RAW/assisted unchanged.

No physical gamepad. Mapper coverage lives in tests/test_human_mapper.mjs.
"""
import inspect
import json
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
from control_console import HUMAN_CMD_STALE_S, Controller
from harness import KP_Z, KD_Z, MAX_FORCE
from assisted_stand import AssistedStandController


def harness_controller():
    c = Controller(harness=True)
    c.handle({'op': 'mode', 'value': 'dynamic'})
    c.enable_harness(seed=True)
    c.handle({'op': 'resume'})
    return c


class HumanControlTests(unittest.TestCase):
    def test_runtime_and_report(self):
        node = subprocess.run(
            ['node', '--disable-warning=MODULE_TYPELESS_PACKAGE_JSON', str(ROOT / 'tests/test_human_mapper.mjs')],
            capture_output=True, text=True, cwd=ROOT)
        if node.returncode != 0 and 'disable-warning' in (node.stderr or ''):
            node = subprocess.run(
                ['node', str(ROOT / 'tests/test_human_mapper.mjs')],
                capture_output=True, text=True, cwd=ROOT)
        line = next((ln for ln in node.stdout.splitlines() if ln.startswith('HUMAN_RESULT ')), '')
        js = json.loads(line[len('HUMAN_RESULT '):]) if line else {'all_pass': False, 'stdout': node.stdout, 'stderr': node.stderr}
        self.assertEqual(node.returncode, 0, node.stderr or node.stdout)
        self.assertTrue(js.get('all_pass'))

        passes = {}
        numbers = dict(js.get('numbers') or {})

        # 8 and 9: 33-vector reaches ctrl; harness wrench stays independent.
        c = harness_controller()
        self.assertTrue(c.harness.enabled)
        self.assertEqual(int(c.m.nu), 33)
        ranges = np.array(c.m.actuator_ctrlrange, dtype=float).copy()
        aid = int(c.m.actuator('shoulder_l_x_motor').id)
        hi = float(c.m.actuator_ctrlrange[aid, 1])
        lo = float(c.m.actuator_ctrlrange[aid, 0])
        u = [0.0] * 33
        u[aid] = hi * 0.10
        qpos = c.d.qpos.copy()
        qvel = c.d.qvel.copy()
        self.assertTrue(c.ingest_human_cmd({'op': 'human_cmd', 'u': u, 't': 1}, now=0.0))
        np.testing.assert_array_equal(c.d.qpos, qpos)
        np.testing.assert_array_equal(c.d.qvel, qvel)
        np.testing.assert_allclose(c.d.ctrl, u)
        np.testing.assert_allclose(c.human_u, u)
        np.testing.assert_array_equal(c.m.actuator_ctrlrange, ranges)
        # Over-range clips to the real ctrlrange; qpos still untouched.
        hot = [0.0] * 33
        hot[aid] = hi * 10
        self.assertTrue(c.ingest_human_cmd({'op': 'human_cmd', 'u': hot, 't': 2}, now=0.01))
        self.assertAlmostEqual(float(c.d.ctrl[aid]), hi)
        np.testing.assert_array_equal(c.d.qpos, qpos)
        # Restore the 10% command used for the step check.
        self.assertTrue(c.ingest_human_cmd({'op': 'human_cmd', 'u': u, 't': 3}, now=0.02))
        peak = 0.0
        for _ in range(400):
            c.tick()
            peak = max(peak, float(np.max(np.abs(c.d.xfrc_applied[c.harness.pelvis_id]))))
        self.assertTrue(c.harness.enabled)
        self.assertGreater(peak, 1.0)
        self.assertAlmostEqual(float(c.d.ctrl[aid]), hi * 0.10)
        self.assertTrue(np.all(c.d.ctrl[np.arange(33) != aid] == 0))
        passes['8_vector_reaches_ctrl'] = True
        passes['9_harness_independent'] = True
        numbers['nu'] = 33
        numbers['shoulder_l_x_id'] = aid
        numbers['shoulder_l_x_ctrl_lo'] = lo
        numbers['shoulder_l_x_ctrl_hi'] = hi
        numbers['ctrl_at_10pct'] = float(c.d.ctrl[aid])
        numbers['xfrc_peak_abs'] = peak
        numbers['harness_enabled_after_steps'] = bool(c.harness.enabled)
        numbers['pelvis_id'] = int(c.harness.pelvis_id)

        # 10: stale timeout zeros human ctrl; harness stays on and still applies xfrc.
        self.assertFalse(c.poll_human_stale(now=0.02 + HUMAN_CMD_STALE_S))
        self.assertTrue(c.poll_human_stale(now=0.02 + HUMAN_CMD_STALE_S + 1e-3))
        np.testing.assert_array_equal(c.d.ctrl, np.zeros(33))
        np.testing.assert_array_equal(c.human_u, np.zeros(33))
        self.assertTrue(c.harness.enabled)
        c.d.ctrl[0] = 5.0  # a stray write must not latch; tick copies the zeroed human vector
        c.tick()
        np.testing.assert_array_equal(c.d.ctrl, np.zeros(33))
        self.assertGreater(float(np.max(np.abs(c.d.xfrc_applied[c.harness.pelvis_id]))), 1.0)
        self.assertTrue(c.harness.enabled)
        passes['10_stale_zeros_human_harness_stays'] = True
        numbers['stale_s'] = HUMAN_CMD_STALE_S
        numbers['ctrl_after_stale'] = 0.0

        # Bad messages do not change the zeroed vector or state.
        self.assertFalse(c.ingest_human_cmd({'op': 'human_cmd', 'u': [0.0] * 32}, now=10))
        self.assertFalse(c.ingest_human_cmd({'op': 'human_cmd', 'u': [True] * 33}, now=10))
        np.testing.assert_array_equal(c.d.ctrl, np.zeros(33))
        np.testing.assert_array_equal(c.d.qpos, c.d.qpos)

        # 11: RAW does not require human_cmd. Assisted still runs. Harness gains untouched.
        raw = Controller()
        self.assertFalse(raw.harness.enabled)
        raw.handle({'op': 'mode', 'value': 'dynamic'})
        raw.handle({'op': 'resume'})
        for _ in range(5):
            raw.tick()
        self.assertGreater(raw.d.time, 0)
        self.assertFalse(raw._human_stream)
        before = raw.d.ctrl.copy()
        qpos = raw.d.qpos.copy()
        self.assertFalse(raw.ingest_human_cmd({'op': 'human_cmd', 'u': [1.0] * 33, 't': 0}))
        np.testing.assert_array_equal(raw.d.ctrl, before)
        np.testing.assert_array_equal(raw.d.qpos, qpos)
        raw.handle({'op': 'torque', 'id': 0, 'value': 5})
        self.assertEqual(float(raw.d.ctrl[0]), 5.0)
        assisted = Controller()
        assisted.handle({'op': 'mode', 'value': 'assisted'})
        self.assertTrue(assisted.assisted)
        self.assertIsInstance(assisted.assist, AssistedStandController)
        self.assertFalse(assisted.harness.enabled)
        self.assertFalse(assisted.ingest_human_cmd({'op': 'human_cmd', 'u': [0.0] * 33}))
        passes['11_raw_and_assisted_unchanged'] = True
        numbers['kp_z'] = KP_Z
        numbers['kd_z'] = KD_Z
        numbers['max_force'] = MAX_FORCE
        self.assertEqual(KP_Z, 30000.0)

        # 14: human ingest/poll source does not assign qpos/qvel. JS already checked.
        for fn in (Controller.ingest_human_cmd, Controller.poll_human_stale, Controller.stage_human_cmd):
            src = inspect.getsource(fn)
            self.assertNotIn('.qpos', src)
            self.assertNotIn('.qvel', src)
            self.assertNotIn('qpos', src)
            self.assertNotIn('qvel', src)
        human_js = ''.join(p.read_text() for p in (ROOT / 'viewer/human').glob('*.js'))
        self.assertNotIn('qpos', human_js)
        self.assertNotIn('qvel', human_js)
        passes['14_no_qpos_qvel_write'] = True

        launch = (ROOT / 'scripts/launch.py').read_text()
        self.assertIn("args.mode == 'harness'", launch)
        self.assertIn("'experiment'", launch)
        self.assertNotIn('human_cmd', launch)
        harness_src = (ROOT / 'physics/harness.py').read_text()
        self.assertIn('KP_Z = 30000.0', harness_src)
        self.assertNotIn('human_cmd', harness_src)
        passes['harness_module_unchanged_spotcheck'] = True

        # Stage/take does not touch ctrl (handler contract).
        staged = Controller(harness=True)
        staged.stage_human_cmd({'op': 'human_cmd', 'u': [0.0] * 33})
        np.testing.assert_array_equal(staged.d.ctrl, np.zeros(33))
        self.assertEqual(staged.take_staged_human()['op'], 'human_cmd')
        self.assertIsNone(staged.take_staged_human())

        report = {
            'all_pass': all(passes.values()) and bool(js.get('all_pass')),
            'passes': {**{k: True for k in js['passes']}, **passes},
            'js_passes': js['passes'],
            'numbers': numbers,
            'contract': {
                'op': 'human_cmd',
                'fields': ['op', 'u', 't'],
                'u_length': 33,
                'example': {'op': 'human_cmd', 'u': [0.0] * 33, 't': 0},
                'hz': 50,
                'stale_s': HUMAN_CMD_STALE_S,
                'deadzone': 0.08,
                'authority_default': 0.10,
                'authority_levels': [0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1],
                'mapping_config': 'viewer/human/gamepad-map.js',
            },
            'gamepad_hardware_validated': False,
            'assumptions': [
                'Standard Gamepad button indices (L1=4, R1=5, L2=6, R2=7, D-pad 12-15) are unverified on a physical DualShock.',
                'AXIS_Y_SIGN = -1 assumes stick up reports -1. Unverified on hardware.',
                'Runtime stale time is server monotonic receive time, not the client t field.',
                'Ankle and wrist hinges are separate 1-DOF cycle entries (real joint names), not a synthetic XYZ group. Vertical stick drives the only actuator.',
                'Authority steps clamp at 5% and 100%; they do not wrap.',
                'In HARNESS, after the first accepted human_cmd, each physics tick copies human_u onto ctrl.',
            ],
        }
        out = ROOT / 'reports/human_control/validation.json'
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + '\n')
        self.assertTrue(report['all_pass'])
        self.assertGreater(numbers['xfrc_peak_abs'], 1.0)
        self.assertAlmostEqual(numbers['ctrl_at_10pct'], hi * 0.10)


if __name__ == '__main__':
    unittest.main()
