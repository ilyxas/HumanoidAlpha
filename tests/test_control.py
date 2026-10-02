import sys
import json
import math
import unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'physics'))
from control_console import Controller
from bridge_full.mapping_full import angular_error_rad

class ControlTests(unittest.TestCase):
    def setUp(self): self.c=Controller()
    def test_discovery(self):
        c=self.c
        self.assertEqual(len(c.inventory['joints']),c.m.njnt)
        self.assertEqual(len(c.inventory['actuators']),c.m.nu)
        self.assertTrue(all(a['controllable'] for a in c.inventory['actuators']))
        self.assertTrue(all(j['visual_bones'] for j in c.inventory['joints']))
    def test_every_joint_independent_and_mapper_response(self):
        c=self.c
        results=[]
        for j in c.inventory['joints']:
            for axis in (range(3) if j['type'] in ('ball','free') else [0]):
                c.handle({'op':'reset'})
                # Isolate each DOF in free space: rotating the root on the
                # floor is now correctly blocked by foot/floor contact.
                root = c.inventory['joints'][0]
                lifted = c.d.qpos[:7].copy(); lifted[2] += .3
                c.handle({'op':'joint','id':root['id'],'value':lifted.tolist()})
                before=c.d.qpos.copy(); pose0=c.snapshot()['pose']; t=c.d.time
                angle=.15
                # Test left hip abduction; positive Y adducts into the other leg.
                if j['name']=='hip_l' and axis==1: angle=-angle
                if j['type']=='hinge': value=[min(j['range'][1],max(j['range'][0],angle))]
                else:
                    q=[math.cos(angle/2),0,0,0];q[axis+1]=math.sin(angle/2)
                    value=list(before[j['qpos_address']:j['qpos_address']+3])+q if j['type']=='free' else q
                c.handle({'op':'joint','id':j['id'],'value':value})
                a,n=j['qpos_address'],j['nq']
                np.testing.assert_array_equal(c.d.qpos[:a],before[:a])
                np.testing.assert_array_equal(c.d.qpos[a+n:],before[a+n:])
                self.assertEqual(c.d.time,t)
                pose=c.snapshot()['pose']
                errors={b:angular_error_rad(pose0['bone_world_quat'][b],pose['bone_world_quat'][b]) for b in j['visual_bones']}
                self.assertGreater(max(errors.values()),.01,j['name'])
                results.append(dict(joint=j['name'],axis=axis,mapped_rotation_change_rad=errors,anatomical_visual_status='NOT VERIFIED'))
        (ROOT/'reports/control_joint_sweep.json').write_text(json.dumps(results,indent=2))
    def test_kinematic_time_and_state_frozen(self):
        before=self.c.d.qpos.copy()
        for _ in range(100):self.c.tick()
        np.testing.assert_array_equal(before,self.c.d.qpos)
        self.assertEqual(self.c.d.time,0)
    def test_invalid_commands_atomic(self):
        c=self.c
        ball=next(j for j in c.inventory['joints'] if j['type']=='ball')
        hinge=next(j for j in c.inventory['joints'] if j['type']=='hinge')
        commands=[{'op':'joint','id':ball['id'],'value':[0,0,0,0]},
            {'op':'joint','id':ball['id'],'value':[0,1,0,0]},
            {'op':'joint','id':hinge['id'],'value':float('nan')},
            {'op':'joint','id':hinge['id'],'value':1000},
            {'op':'joint','id':-1,'value':0}, {'op':'mode','value':'bad'}]
        before=c.d.qpos.copy()
        for cmd in commands:
            with self.assertRaises(ValueError):c.handle(cmd)
            np.testing.assert_array_equal(before,c.d.qpos)
    def test_quaternion_normalization(self):
        c=self.c;j=next(j for j in c.inventory['joints'] if j['type']=='ball')
        c.handle({'op':'joint','id':j['id'],'value':[2,.1,0,0]})
        a=j['qpos_address'];self.assertAlmostEqual(np.linalg.norm(c.d.qpos[a:a+4]),1)
    def test_all_actuators_dynamic_response(self):
        c=self.c
        for actuator in c.inventory['actuators']:
            c.handle({'op':'reset'});c.handle({'op':'mode','value':'dynamic'});c.handle({'op':'resume'})
            c.tick();baseline=c.d.qvel.copy()
            c.handle({'op':'reset'});c.handle({'op':'mode','value':'dynamic'})
            c.handle({'op':'torque','id':actuator['id'],'value':1});c.handle({'op':'resume'});c.tick()
            self.assertGreater(np.linalg.norm(c.d.qvel-baseline),1e-7,actuator['name'])
            self.assertGreater(c.d.time,0)
    def test_pause_zero_transition_and_reset(self):
        c=self.c;reference=c.d.qpos.copy()
        c.handle({'op':'mode','value':'dynamic'});c.handle({'op':'torque','id':0,'value':5})
        c.handle({'op':'resume'});c.tick();c.handle({'op':'pause'});t=c.d.time;c.tick();self.assertEqual(t,c.d.time)
        c.handle({'op':'zero'});self.assertTrue(c.paused);self.assertTrue(np.all(c.d.ctrl==0))
        c.handle({'op':'mode','value':'kinematic'});self.assertTrue(np.all(c.d.qvel==0))
        c.handle({'op':'reset'});np.testing.assert_array_equal(c.d.qpos,reference);self.assertEqual(c.d.time,0)
    def test_dynamic_rejects_position_kinematic_rejects_torque(self):
        with self.assertRaises(ValueError):self.c.handle({'op':'torque','id':0,'value':0})
        self.c.handle({'op':'mode','value':'dynamic'})
        with self.assertRaises(ValueError):self.c.handle({'op':'joint','id':0,'value':[0,0,1,1,0,0,0]})
    def test_torque_limits(self):
        c=self.c;c.handle({'op':'mode','value':'dynamic'})
        for value in [float('inf'),float('nan'),1e9]:
            with self.assertRaises(ValueError):c.handle({'op':'torque','id':0,'value':value})
        self.assertTrue(np.all(c.d.ctrl==0))

if __name__=='__main__':unittest.main(verbosity=2)
