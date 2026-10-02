"""Stage 1: Audit export-copy blend + export Xandra.glb (nude kit + armature).
Runs ONLY on Xandra_F4_BodyLab_export_src.blend — never saves originals.
"""
from __future__ import annotations
import json, os, sys
from datetime import datetime
from pathlib import Path

import bpy
from mathutils import Vector

V02 = Path(__file__).resolve().parents[1]
BLEND = V02 / "Xandra_F4_BodyLab_export_src.blend"
AUDIT_JSON = V02 / "reports" / "rig_audit.json"
GLB_OUT = V02 / "assets" / "Xandra.glb"
EXPORT_LOG = V02 / "reports" / "export_log.json"

ARM = "root"
ALWAYS_HIDE = [
    "F3_FaceShapes", "F4_BodyShapes",
    "UE_Bikini_Top", "UE_Bikini_Legs_F4",
    "F4_Torso_Censored", "F4_Legs_Censored",
    "Yuki_Bangs.001", "Earrings_Evie",
]
NUDE_SHOW = [
    "F4_Torso", "F4_Legs_Nude.001", "F4_Head", "F4_Hands", "F4_Feet",
    "Hair 2_Back.001", "Yuki_Bangs", "Earrings_CircleDrop", "stage",
]


def ensure_loaded():
    if bpy.data.filepath != str(BLEND) and not bpy.data.objects.get(ARM):
        bpy.ops.wm.open_mainfile(filepath=str(BLEND))


def bone_hierarchy_sample(arm, max_depth=4, max_nodes=40):
    bones = arm.data.bones
    roots = [b for b in bones if b.parent is None]
    out = []

    def walk(b, depth):
        if len(out) >= max_nodes:
            return
        out.append({
            "name": b.name,
            "depth": depth,
            "parent": b.parent.name if b.parent else None,
            "use_deform": bool(b.use_deform),
            "head_local": list(b.head_local),
            "tail_local": list(b.tail_local),
            "length": float(b.length),
            "children_n": len(b.children),
        })
        if depth < max_depth:
            for c in b.children:
                walk(c, depth + 1)

    for r in roots:
        walk(r, 0)
    return out


def audit():
    arm = bpy.data.objects.get(ARM)
    scene = bpy.context.scene
    unit = scene.unit_settings

    report = {
        "blend": str(BLEND),
        "blend_size": os.path.getsize(BLEND),
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S IDT"),
        "blender_version": bpy.app.version_string,
        "units": {
            "system": unit.system,
            "scale_length": float(unit.scale_length),
            "length_unit": unit.length_unit,
        },
        "armature": None,
        "objects": [],
        "meshes_with_armature_mod": [],
        "materials_sample": [],
        "drivers_count": 0,
        "constraints_on_bones": [],
        "shape_keys": [],
        "notes": [],
    }

    if arm is None:
        report["notes"].append("BLOCKER: armature 'root' not found")
        return report

    mw = arm.matrix_world
    report["armature"] = {
        "name": arm.name,
        "location": list(arm.location),
        "rotation_mode": arm.rotation_mode,
        "rotation_quaternion": list(arm.rotation_quaternion) if arm.rotation_mode == "QUATERNION" else None,
        "rotation_euler": list(arm.rotation_euler),
        "scale": list(arm.scale),
        "matrix_world_diag_scale": [float(mw.to_scale()[i]) for i in range(3)],
        "bone_count_all": len(arm.data.bones),
        "bone_count_deform": sum(1 for b in arm.data.bones if b.use_deform),
        "pose_bone_count": len(arm.pose.bones),
        "hierarchy_sample": bone_hierarchy_sample(arm),
        "bone_names": [b.name for b in arm.data.bones],
    }

    # Bone constraints
    for pb in arm.pose.bones:
        for c in pb.constraints:
            report["constraints_on_bones"].append({
                "bone": pb.name, "type": c.type, "name": c.name, "mute": bool(c.mute),
            })

    # Drivers on armature / bones
    if arm.animation_data and arm.animation_data.drivers:
        report["drivers_count"] += len(arm.animation_data.drivers)
    if arm.data.animation_data and arm.data.animation_data.drivers:
        report["drivers_count"] += len(arm.data.animation_data.drivers)

    for obj in bpy.data.objects:
        entry = {
            "name": obj.name,
            "type": obj.type,
            "hide_viewport": bool(obj.hide_viewport),
            "hide_render": bool(obj.hide_render),
            "parent": obj.parent.name if obj.parent else None,
            "modifiers": [{"name": m.name, "type": m.type} for m in obj.modifiers],
            "vertex_groups_n": len(obj.vertex_groups) if hasattr(obj, "vertex_groups") else 0,
            "materials": [s.material.name if s.material else None for s in (obj.material_slots or [])],
        }
        report["objects"].append(entry)

        if obj.type == "MESH":
            has_arm = any(m.type == "ARMATURE" for m in obj.modifiers)
            if has_arm:
                arm_mods = []
                for m in obj.modifiers:
                    if m.type == "ARMATURE":
                        arm_mods.append({
                            "name": m.name,
                            "object": m.object.name if m.object else None,
                            "use_vertex_groups": bool(m.use_vertex_groups),
                            "use_bone_envelopes": bool(m.use_bone_envelopes),
                        })
                me = obj.data
                sk = None
                if me.shape_keys:
                    sk = {
                        "n_keys": len(me.shape_keys.key_blocks),
                        "names": [kb.name for kb in me.shape_keys.key_blocks][:40],
                    }
                    report["shape_keys"].append({"object": obj.name, **sk})
                # sample weight presence
                vg_names = [vg.name for vg in obj.vertex_groups][:30]
                n_verts = len(me.vertices)
                weighted = 0
                sample_n = min(n_verts, 500)
                for i in range(sample_n):
                    if me.vertices[i].groups:
                        weighted += 1
                report["meshes_with_armature_mod"].append({
                    "name": obj.name,
                    "verts": n_verts,
                    "polys": len(me.polygons),
                    "armature_mods": arm_mods,
                    "vertex_groups_n": len(obj.vertex_groups),
                    "vertex_groups_sample": vg_names,
                    "shape_keys": sk,
                    "weight_sample_nonzero_frac": weighted / sample_n if sample_n else 0,
                    "in_nude_show": obj.name in NUDE_SHOW,
                    "in_always_hide": obj.name in ALWAYS_HIDE,
                })
                if obj.animation_data and obj.animation_data.drivers:
                    report["drivers_count"] += len(obj.animation_data.drivers)
                if me.animation_data and me.animation_data.drivers:
                    report["drivers_count"] += len(me.animation_data.drivers)

    # Materials / textures sample
    for mat in list(bpy.data.materials)[:40]:
        tex_nodes = []
        if mat.use_nodes and mat.node_tree:
            for n in mat.node_tree.nodes:
                if n.type == "TEX_IMAGE" and n.image:
                    tex_nodes.append(n.image.name)
        report["materials_sample"].append({
            "name": mat.name, "images": tex_nodes[:8],
        })

    # Key mapped bones present?
    needed = [
        "pelvis", "spine_01", "spine_05", "head",
        "thigh_r", "calf_r", "foot_r",
        "upperarm_r", "lowerarm_r", "hand_r",
        "clavicle_r",
    ]
    names = set(b.name for b in arm.data.bones)
    report["mapped_bones_present"] = {n: (n in names) for n in needed}
    report["missing_mapped"] = [n for n in needed if n not in names]
    return report


def prepare_export_visibility():
    """Hide censored/bikini; show nude kit. Deselect all; select export set."""
    # Ensure OBJECT mode + valid view layer context (background -b)
    for area in bpy.context.screen.areas if bpy.context.screen else []:
        pass
    try:
        bpy.ops.object.mode_set(mode="OBJECT")
    except Exception:
        pass

    for name in ALWAYS_HIDE:
        o = bpy.data.objects.get(name)
        if o:
            o.hide_viewport = True
            o.hide_render = True
            try:
                o.hide_set(True)
            except Exception:
                pass
    for name in NUDE_SHOW:
        o = bpy.data.objects.get(name)
        if o:
            o.hide_viewport = False
            o.hide_render = False
            try:
                o.hide_set(False)
            except Exception:
                pass

    # Manual deselect (ops may fail in -b without view3d)
    for o in bpy.data.objects:
        o.select_set(False)

    arm = bpy.data.objects[ARM]
    try:
        arm.hide_set(False)
    except Exception:
        pass
    arm.hide_viewport = False
    arm.select_set(True)
    selected = [ARM]
    for name in NUDE_SHOW:
        o = bpy.data.objects.get(name)
        if o is None:
            continue
        if o.type == "MESH" or name == "stage":
            o.select_set(True)
            selected.append(name)
    bpy.context.view_layer.objects.active = arm
    return selected


def export_glb(selected):
    GLB_OUT.parent.mkdir(parents=True, exist_ok=True)
    # Documented transform choice: glTF expects Y-up. Blender is Z-up.
    # Use exporter convert; do NOT bake scale on armature manually.
    # export_apply=False to keep skinning bind pose intact.
    kwargs = dict(
        filepath=str(GLB_OUT),
        use_selection=True,
        export_format="GLB",
        export_texcoords=True,
        export_normals=True,
        export_materials="EXPORT",
        export_colors=True,
        export_attributes=True,
        export_cameras=False,
        export_lights=False,
        export_yup=True,
        export_apply=False,
        export_animations=False,
        export_skins=True,
        export_morph=True,  # shape keys if any — may be limited
        export_def_bones=False,  # export all bones that influence; Blender 4.x
        export_rest_position_armature=True,
    )
    # Blender 4.3 may use slightly different kwargs — try/adapt
    try:
        bpy.ops.export_scene.gltf(**kwargs)
    except TypeError as e:
        # Remove unknown kwargs and retry
        # Probe signature via retry with minimal set
        minimal = dict(
            filepath=str(GLB_OUT),
            use_selection=True,
            export_format="GLB",
            export_yup=True,
            export_apply=False,
            export_skins=True,
            export_animations=False,
            export_materials="EXPORT",
            export_texcoords=True,
            export_normals=True,
        )
        # Try adding optional flags one by one
        for k, v in kwargs.items():
            if k in minimal:
                continue
            try:
                bpy.ops.export_scene.gltf(**{**minimal, k: v})
                minimal[k] = v
            except TypeError:
                pass
        bpy.ops.export_scene.gltf(**minimal)
        kwargs = minimal

    return {
        "path": str(GLB_OUT),
        "exists": GLB_OUT.exists(),
        "size": GLB_OUT.stat().st_size if GLB_OUT.exists() else 0,
        "selected": selected,
        "export_kwargs": {k: (v if not callable(v) else str(v)) for k, v in kwargs.items()},
        "axes_note": "export_yup=True: Blender Z-up → glTF Y-up conversion by exporter",
        "apply_note": "export_apply=False: preserve armature bind / skin weights",
        "transforms_on_copy": "visibility hide/show only; no armature scale/location edits",
    }


def main():
    ensure_loaded()
    report = audit()
    AUDIT_JSON.parent.mkdir(parents=True, exist_ok=True)
    AUDIT_JSON.write_text(json.dumps(report, indent=2))
    print("AUDIT_WRITTEN", AUDIT_JSON)

    if report.get("missing_mapped"):
        print("BLOCKER_MISSING_BONES", report["missing_mapped"])
    if not report.get("meshes_with_armature_mod"):
        print("BLOCKER_NO_SKINNED_MESHES")
        EXPORT_LOG.write_text(json.dumps({"status": "BLOCKED", "reason": "no skinned meshes"}, indent=2))
        return

    selected = prepare_export_visibility()
    elog = export_glb(selected)
    elog["audit_skinned_nude"] = [
        m["name"] for m in report["meshes_with_armature_mod"] if m.get("in_nude_show")
    ]
    EXPORT_LOG.write_text(json.dumps(elog, indent=2))
    print("EXPORT_DONE", json.dumps({
        "size": elog["size"], "exists": elog["exists"],
        "selected_n": len(selected),
    }))


if __name__ == "__main__":
    main()
