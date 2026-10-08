"""Animation mode (EXP002.1-B): node unit tests for the locomotion controller and
stick mapping, plus a structural check of the authoritative GLB + manifest."""
import json
from pathlib import Path
import struct
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
CLIPS = ['Idle', 'Walk_Fwd', 'Run_Fwd', 'Walk_Back', 'Squat', 'Jump', 'ToPlank', 'ToBridge']


def run_node(script):
    command = ['node', '--disable-warning=MODULE_TYPELESS_PACKAGE_JSON', str(ROOT / 'tests' / script)]
    result = subprocess.run(command, capture_output=True, text=True, cwd=ROOT)
    if result.returncode != 0 and 'disable-warning' in (result.stderr or ''):
        result = subprocess.run(command[:1] + command[2:], capture_output=True, text=True, cwd=ROOT)
    return result


class AnimationModeTests(unittest.TestCase):
    def test_locomotion_controller(self):
        result = run_node('test_locomotion.mjs')
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        self.assertIn('test_locomotion OK', result.stdout)

    def test_anim_controller_one_shots(self):
        result = run_node('test_anim_controller.mjs')
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        self.assertIn('test_anim_controller OK', result.stdout)

    def test_walk_control(self):
        result = run_node('test_walk_control.mjs')
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        self.assertIn('test_walk_control OK', result.stdout)

    def test_glb_and_manifest_agree(self):
        manifest = json.loads((ROOT / 'assets/Xandra_Animated.manifest.json').read_text())
        with open(ROOT / 'assets/Xandra_Animated.glb', 'rb') as handle:
            magic, _version, _length, chunk_length, _chunk_type = struct.unpack('<4sIIII', handle.read(20))
            self.assertEqual(magic, b'glTF')
            gltf = json.loads(handle.read(chunk_length))
        names = sorted(a['name'] for a in gltf['animations'])
        self.assertEqual(names, sorted(CLIPS))
        self.assertEqual(sorted(manifest['clips']), sorted(CLIPS))
        accessors = gltf['accessors']
        for animation in gltf['animations']:
            duration = max(accessors[s['input']]['max'][0] for s in animation['samplers'])
            self.assertAlmostEqual(duration, manifest['clips'][animation['name']]['duration_s'], places=3)
        heads = [m for m in gltf['meshes'] if (m.get('extras') or {}).get('targetNames')]
        self.assertEqual(len(heads), 1)
        self.assertEqual(len(heads[0]['extras']['targetNames']), 52)
        self.assertEqual(manifest['global_info']['morph_targets']['count'], 52)
        self.assertEqual(len(gltf['skins'][0]['joints']), manifest['global_info']['joint_count'])
        for name in ('Idle', 'Walk_Fwd', 'Run_Fwd', 'Walk_Back'):
            self.assertTrue(manifest['clips'][name]['loop'])
        for name in ('Squat', 'Jump', 'ToPlank', 'ToBridge'):
            self.assertFalse(manifest['clips'][name]['loop'])


if __name__ == '__main__':
    unittest.main()
