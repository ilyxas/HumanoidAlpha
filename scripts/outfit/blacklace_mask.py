"""Hide Xandra's skin under the opaque Black Lace garments (corset, briefs).

blender -b SRC.blend --python scripts/outfit/blacklace_mask.py -- DST.blend

Body vertices closer than NEAR to an opaque garment surface, and whose nearest
garment point lies at least EDGE away from that garment's open border, go into
the vertex group BL_Hidden. The existing Clothes_Mask modifiers are retargeted
to it and re-enabled (the original Clothes_Hidden group is kept).
"""
import sys
import bpy
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

OPAQUE = ['BL_Corset', 'BL_HighWaist_Briefs']
BODY = ['F4_Torso', 'F4_Legs_Nude.001']
GROUP = 'BL_Hidden'
NEAR = 0.02   # m, body-to-garment distance in rest pose
EDGE = 0.012  # m, keep skin this close to a garment border visible


def garment(name):
    ob = bpy.data.objects[name]
    mw = ob.matrix_world
    me = ob.data  # rest pose == mesh data (armature is the only deform before solidify)
    verts = [mw @ v.co for v in me.vertices]
    bvh = BVHTree.FromPolygons(verts, [tuple(p.vertices) for p in me.polygons])
    count = {}
    for p in me.polygons:
        for k in p.edge_keys:
            count[k] = count.get(k, 0) + 1
    border = {i for k, c in count.items() if c == 1 for i in k}
    kd = KDTree(len(border))
    for i in border:
        kd.insert(verts[i], i)
    kd.balance()
    return bvh, kd


def main(dst):
    arm = bpy.data.objects['root']
    pose = arm.data.pose_position
    arm.data.pose_position = 'REST'
    garments = [garment(n) for n in OPAQUE]
    for name in BODY:
        ob = bpy.data.objects[name]
        masks = [m for m in ob.modifiers if m.type == 'MASK']
        for m in masks:
            m.show_viewport = m.show_render = False
        dg = bpy.context.evaluated_depsgraph_get()
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        assert len(me.vertices) == len(ob.data.vertices)
        hid = []
        for v in me.vertices:
            p = ob.matrix_world @ v.co
            for bvh, kd in garments:
                loc, _, _, d = bvh.find_nearest(p)
                if loc is not None and d < NEAR and kd.find(loc)[2] > EDGE:
                    hid.append(v.index)
                    break
        ev.to_mesh_clear()
        g = ob.vertex_groups.get(GROUP) or ob.vertex_groups.new(name=GROUP)
        g.remove(list(range(len(ob.data.vertices))))
        g.add(hid, 1.0, 'REPLACE')
        for m in masks:
            m.vertex_group = GROUP
            m.invert_vertex_group = True
            m.show_viewport = m.show_render = True
        print('BL_MASK', name, len(hid), '/', len(ob.data.vertices))
    arm.data.pose_position = pose
    bpy.ops.wm.save_as_mainfile(filepath=dst, compress=True)


main(sys.argv[sys.argv.index('--') + 1])
