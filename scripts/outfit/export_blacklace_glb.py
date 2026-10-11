"""Export the Black Lace fitting blend to a viewer GLB (same layout as Xandra_Animated.glb).

blender -b assets/Xandra_BlackLace_Fitting_v01.blend --python scripts/outfit/export_blacklace_glb.py -- assets/Xandra_BlackLace.glb

Works on the loaded session only (the .blend is not saved):
- keeps the 8 manifest clips (Xandra_<Clip> -> <Clip>), drops other actions/NLA;
- keeps the 52 ARKit morphs on F4_Head, bakes other shape keys to their mix;
- deletes body vertices hidden by the Clothes_Mask modifiers;
- applies garment Solidify in rest pose and joins BL_* objects per material;
- bakes the lace ColorRamp alpha into an RGBA texture and its UV MULTIPLY_ADD
  into the UVs, since glTF cannot express either node.
"""
import sys
import bmesh
import bpy

CLIPS = ['Idle', 'Walk_Fwd', 'Run_Fwd', 'Walk_Back', 'Squat', 'Jump', 'ToPlank', 'ToBridge']
MORPHS = [
    'eyeBlinkLeft', 'eyeLookDownLeft', 'eyeLookInLeft', 'eyeLookOutLeft', 'eyeLookUpLeft', 'eyeSquintLeft',
    'eyeWideLeft', 'eyeBlinkRight', 'eyeLookDownRight', 'eyeLookInRight', 'eyeLookOutRight', 'eyeLookUpRight',
    'eyeSquintRight', 'eyeWideRight', 'jawForward', 'jawLeft', 'jawRight', 'jawOpen', 'mouthClose',
    'mouthFunnel', 'mouthPucker', 'mouthRight', 'mouthLeft', 'mouthSmileLeft', 'mouthSmileLeftold',
    'mouthSmileRightold', 'mouthFrownRight', 'mouthFrownLeft', 'mouthDimpleLeft', 'mouthDimpleRight',
    'mouthStretchLeft', 'mouthStretchRight', 'mouthRollLower', 'mouthRollUpper', 'mouthShrugLower',
    'mouthShrugUpper', 'mouthPressLeft', 'mouthPressRight', 'mouthLowerDownLeft', 'mouthLowerDownRight',
    'mouthUpperUpLeft', 'mouthUpperUpRight', 'browDownLeft', 'browDownRight', 'browInnerUp',
    'browOuterUpLeft', 'browOuterUpRight', 'cheekPuff', 'cheekSquintLeft', 'cheekSquintRight',
    'noseSneerLeft', 'noseSneerRight']
LACE = 'BL_Reference_Lace_Alpha'
LACE_SIZE = 1024


def actions(arm):
    ad = arm.animation_data
    for t in list(ad.nla_tracks):
        ad.nla_tracks.remove(t)
    ad.action = None
    keep = {'Xandra_' + c for c in CLIPS}
    for a in list(bpy.data.actions):
        if a.name in keep:
            a.name = a.name[len('Xandra_'):]
            a.use_fake_user = True
        else:
            bpy.data.actions.remove(a)
    assert sorted(a.name for a in bpy.data.actions) == sorted(CLIPS)


def shape_keys(ob, keep):
    sk = ob.data.shape_keys
    if not sk:
        return
    if not keep:
        ob.shape_key_add(name='__mix', from_mix=True)
        mix = [v.co.copy() for v in sk.key_blocks['__mix'].data]
        ob.shape_key_clear()
        for v, co in zip(ob.data.vertices, mix):
            v.co = co
        return
    for kb in list(sk.key_blocks)[1:]:
        if kb.name not in keep:
            ob.shape_key_remove(kb)
    names = [kb.name for kb in sk.key_blocks][1:]
    assert names == [n for n in MORPHS if n in names] and set(names) == set(MORPHS), names


def apply_masks(ob):
    for m in [m for m in ob.modifiers if m.type == 'MASK']:
        if m.show_render and m.vertex_group in ob.vertex_groups:
            gi = ob.vertex_groups[m.vertex_group].index
            bm = bmesh.new(); bm.from_mesh(ob.data)
            dl = bm.verts.layers.deform.verify()
            hid = [v for v in bm.verts if (v[dl].get(gi, 0.0) > 0.0) == m.invert_vertex_group]
            bmesh.ops.delete(bm, geom=hid, context='VERTS')
            bm.to_mesh(ob.data); bm.free()
            print('MASKED', ob.name, len(hid))
        ob.modifiers.remove(m)


def bake_lace():
    mat = bpy.data.materials.get(LACE)
    nt = mat.node_tree
    tex = next(n for n in nt.nodes if n.type == 'TEX_IMAGE')
    ramp = next(n for n in nt.nodes if n.type == 'VALTORGB').color_ramp
    vm = next(n for n in nt.nodes if n.type == 'VECT_MATH')
    bsdf = next(n for n in nt.nodes if n.type == 'BSDF_PRINCIPLED')
    scale, offset = vm.inputs[1].default_value[:2], vm.inputs[2].default_value[:2]
    src = tex.image.copy(); src.scale(LACE_SIZE, LACE_SIZE)
    px = list(src.pixels)  # linear float RGBA
    base = bsdf.inputs['Base Color'].default_value
    srgb = [c * 12.92 if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055 for c in base[:3]]
    out = [0.0] * len(px)
    for i in range(0, len(px), 4):
        lum = 0.2126 * px[i] + 0.7152 * px[i + 1] + 0.0722 * px[i + 2]
        out[i:i + 4] = (*srgb, ramp.evaluate(lum)[0])
    img = bpy.data.images.new('BL_Lace_Alpha', LACE_SIZE, LACE_SIZE, alpha=True)
    img.pixels = out; img.file_format = 'PNG'; img.pack()
    for n in [n for n in nt.nodes if n not in (bsdf,) and n.type != 'OUTPUT_MATERIAL']:
        nt.nodes.remove(n)
    t = nt.nodes.new('ShaderNodeTexImage'); t.image = img
    nt.links.new(t.outputs['Color'], bsdf.inputs['Base Color'])
    nt.links.new(t.outputs['Alpha'], bsdf.inputs['Alpha'])
    return scale, offset


def main(dst):
    arm = bpy.data.objects['root']
    actions(arm)
    arm.data.pose_position = 'REST'
    for o in list(bpy.data.objects):
        if o.type in ('CAMERA', 'LIGHT') or o.name == 'Xa_Floor':
            bpy.data.objects.remove(o)
    vis = [o for o in bpy.context.view_layer.objects if o.type == 'MESH' and o.visible_get()]
    for o in vis:
        shape_keys(o, MORPHS if o.name == 'F4_Head' else None)
        apply_masks(o)
    scale, offset = bake_lace()
    garments = [o for o in vis if o.name.startswith('BL_')]
    bpy.ops.object.select_all(action='DESELECT')
    for o in garments:
        bpy.context.view_layer.objects.active = o
        for m in [m for m in o.modifiers if m.type == 'SOLIDIFY']:
            bpy.ops.object.modifier_apply(modifier=m.name)
        if o.data.materials[0].name == LACE:
            for uv in o.data.uv_layers.active.data:
                uv.uv = (uv.uv[0] * scale[0] + offset[0], uv.uv[1] * scale[1] + offset[1])
    by_mat = {}
    for o in garments:
        by_mat.setdefault(o.data.materials[0].name, []).append(o)
    for name, objs in by_mat.items():
        bpy.ops.object.select_all(action='DESELECT')
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        if len(objs) > 1:
            bpy.ops.object.join()
        objs[0].name = name.replace('BL_', 'BL_Outfit_')
    arm.data.pose_position = 'POSE'
    bpy.ops.export_scene.gltf(
        filepath=dst, export_format='GLB', use_visible=True,
        export_animation_mode='ACTIONS', export_morph=True, export_skins=True,
        export_influence_nb=4, export_force_sampling=True, export_frame_step=1,
        export_anim_slide_to_zero=True, export_apply=False, export_cameras=False,
        export_lights=False, export_vertex_color='NONE')


main(sys.argv[sys.argv.index('--') + 1])
