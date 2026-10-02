"""Independent GLB bind-frame comparison: not a check against mapper output alone."""
import sys,struct,json,unittest
import math
import mujoco
import numpy as np
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'physics'))
from bridge_full.mapping_full import load_calibration,R_to_quat,quat_mul,quat_conj,angular_error_rad
class GLBFrameTests(unittest.TestCase):
    def test_physical_and_visual_limb_anchors_during_rotation(self):
        from control_console import Controller
        raw=(ROOT/'assets/Xandra.glb').read_bytes()
        nodes=json.loads(raw[20:20+struct.unpack_from('<I',raw,12)[0]])['nodes']
        parents={child:i for i,node in enumerate(nodes) for child in node.get('children',[])}
        c=Controller(); rows=[]
        def rotate(q, v):
            matrix=np.zeros(9)
            mujoco.mju_quat2Mat(matrix,np.asarray(q,dtype=float))
            return matrix.reshape(3,3)@v
        for axis in range(3):
            for angle in (0., .2, .7, 1.5, 2.3):
                c.env.reset()
                for side in ('l','r'):
                    joint=c.m.joint('shoulder_'+side); a=joint.qposadr[0]
                    q=np.array([math.cos(angle/2),0.,0.,0.]);q[axis+1]=math.sin(angle/2)
                    c.d.qpos[a:a+4]=q
                mujoco.mj_forward(c.m,c.d)
                pose=c.snapshot()['pose']; cache={}
                def world(i):
                    if i in cache:return cache[i]
                    node=nodes[i]
                    p,q,s=world(parents[i]) if i in parents else (np.zeros(3),[1,0,0,0],np.ones(3))
                    p=p+rotate(q,s*np.array(node.get('translation',[0,0,0])))
                    s=s*np.array(node.get('scale',[1,1,1]))
                    local=node.get('rotation',[0,0,0,1]);q=quat_mul(q,[local[3],*local[:3]])
                    if node.get('name') in pose['bone_world_quat']:
                        q=quat_mul([2**-.5,-2**-.5,0,0],pose['bone_world_quat'][node['name']])
                    cache[i]=p,q,s
                    return cache[i]
                for i,node in enumerate(nodes):
                    name=node.get('name','')
                    if name not in {f'{segment}_{side}' for segment in
                                    ('upperarm','lowerarm','hand','thigh','calf','foot') for side in ('l','r')}:
                        continue
                    physical=c.d.xpos[c.m.body(name).id]
                    physical=np.array([physical[0],physical[2],-physical[1]])
                    error=float(np.linalg.norm(world(i)[0]-physical))
                    self.assertLess(error,1e-5,(name,axis,angle))
                    rows.append(dict(bone=name,axis=axis,angle=angle,anchor_error_m=error))
        (ROOT/'reports/collision/visual_anchor_validation.json').write_text(json.dumps(rows,indent=2)+'\n')

    def test_exported_rest_frames(self):
        raw=(ROOT/'assets/Xandra.glb').read_bytes()
        glb=json.loads(raw[20:20+struct.unpack_from('<I',raw,12)[0]])
        nodes=glb['nodes'];parents={c:i for i,n in enumerate(nodes) for c in n.get('children',[])}
        def world(i):
            self.assertNotIn('matrix',nodes[i],'Matrix nodes need a separate decomposition')
            q=nodes[i].get('rotation',[0,0,0,1]);q=[q[3],*q[:3]]
            return quat_mul(world(parents[i]),q) if i in parents else q
        C=[2**-.5,-2**-.5,0,0];rows=[]
        for name,bone in load_calibration()['bones'].items():
            i=next(i for i,n in enumerate(nodes) if n.get('name')==name)
            q=R_to_quat(bone['R_rest_world'])
            corrected=angular_error_rad(world(i),quat_mul(C,q))
            previous=angular_error_rad(world(i),quat_mul(quat_mul(C,q),quat_conj(C)))
            self.assertLess(corrected,2e-5,name)
            self.assertGreater(previous,1.5,name)
            rows.append(dict(bone=name,glb_rest_error_corrected_rad=corrected,previous_error_rad=previous))
        (ROOT/'reports/glb_frame_validation.json').write_text(json.dumps(rows,indent=2))
if __name__=='__main__':unittest.main(verbosity=2)
