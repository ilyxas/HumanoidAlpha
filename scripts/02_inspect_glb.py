"""Inspect Xandra.glb: skins, bones, meshes, scale."""
from __future__ import annotations
import json, struct, zipfile
from pathlib import Path
from collections import defaultdict

try:
    from pygltflib import GLTF2
except ImportError:
    GLTF2 = None

V02 = Path(__file__).resolve().parents[1]
GLB = V02 / "assets" / "Xandra.glb"
OUT = V02 / "reports" / "glb_inspect.json"


def accessor_count(gltf, acc_idx):
    if acc_idx is None:
        return 0
    return gltf.accessors[acc_idx].count


def main():
    assert GLB.exists(), GLB
    gltf = GLTF2().load(str(GLB))
    nodes = gltf.nodes or []
    meshes = gltf.meshes or []
    skins = gltf.skins or []
    mats = gltf.materials or []
    imgs = gltf.images or []
    texs = gltf.textures or []

    # Build node tree
    name_to_idx = {n.name: i for i, n in enumerate(nodes) if n.name}
    children_of = defaultdict(list)
    roots = set(range(len(nodes)))
    for i, n in enumerate(nodes):
        for c in (n.children or []):
            children_of[i].append(c)
            roots.discard(c)

    # Find armature-like: skins.joints
    skin_info = []
    all_joint_names = []
    for si, skin in enumerate(skins):
        joints = skin.joints or []
        jnames = [nodes[j].name for j in joints]
        all_joint_names.extend(jnames)
        skin_info.append({
            "index": si,
            "name": skin.name,
            "n_joints": len(joints),
            "joints_sample": jnames[:40],
            "has_ibm": skin.inverseBindMatrices is not None,
            "skeleton_root": nodes[skin.skeleton].name if skin.skeleton is not None else None,
        })

    mesh_info = []
    skinned_prims = 0
    for mi, mesh in enumerate(meshes):
        prims = []
        for pi, p in enumerate(mesh.primitives or []):
            attrs = p.attributes
            joints = getattr(attrs, "JOINTS_0", None)
            weights = getattr(attrs, "WEIGHTS_0", None)
            pos = getattr(attrs, "POSITION", None)
            is_skinned = joints is not None and weights is not None
            if is_skinned:
                skinned_prims += 1
            prims.append({
                "attrs": {k: getattr(attrs, k) for k in ("POSITION","NORMAL","TEXCOORD_0","JOINTS_0","WEIGHTS_0") if getattr(attrs, k, None) is not None},
                "skinned": is_skinned,
                "n_verts": accessor_count(gltf, pos) if pos is not None else 0,
                "material": p.material,
                "mode": p.mode,
                "targets_n": len(p.targets) if p.targets else 0,
            })
        # which nodes use this mesh
        users = [nodes[i].name for i, n in enumerate(nodes) if n.mesh == mi]
        mesh_info.append({
            "index": mi,
            "name": mesh.name,
            "n_primitives": len(prims),
            "primitives": prims,
            "node_users": users,
            "skin_on_nodes": [
                nodes[i].skin for i, n in enumerate(nodes) if n.mesh == mi
            ],
        })

    # Check key bones
    needed = [
        "pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05",
        "neck_01", "neck_02", "head",
        "thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r",
        "clavicle_l", "clavicle_r",
        "upperarm_l", "upperarm_r", "lowerarm_l", "lowerarm_r",
        "hand_l", "hand_r",
    ]
    joint_set = set(all_joint_names)
    node_set = set(n.name for n in nodes if n.name)
    bone_check = {b: {"in_nodes": b in node_set, "in_joints": b in joint_set} for b in needed}

    # Scale / translation on root-ish nodes
    root_nodes = []
    for i in sorted(roots):
        n = nodes[i]
        root_nodes.append({
            "name": n.name,
            "translation": n.translation,
            "rotation": n.rotation,
            "scale": n.scale,
            "mesh": n.mesh,
            "skin": n.skin,
            "n_children": len(n.children or []),
        })

    # Sample hierarchy under first skin skeleton
    def walk(idx, depth, acc, limit=50):
        if len(acc) >= limit:
            return
        n = nodes[idx]
        acc.append({"name": n.name, "depth": depth, "scale": n.scale, "translation": n.translation})
        for c in (n.children or []):
            walk(c, depth + 1, acc, limit)

    hierarchy = []
    if skins and skins[0].skeleton is not None:
        walk(skins[0].skeleton, 0, hierarchy)
    elif skins and skins[0].joints:
        # find parentless among joints
        jset = set(skins[0].joints)
        for j in skins[0].joints:
            # if no parent in joints list via children links inverse
            parents = [i for i, n in enumerate(nodes) if n.children and j in n.children]
            if not parents or parents[0] not in jset:
                walk(j, 0, hierarchy)
                break

    report = {
        "glb_path": str(GLB),
        "glb_size": GLB.stat().st_size,
        "n_nodes": len(nodes),
        "n_meshes": len(meshes),
        "n_skins": len(skins),
        "n_materials": len(mats),
        "n_images": len(imgs),
        "n_textures": len(texs),
        "skinned_primitives": skinned_prims,
        "skins": skin_info,
        "meshes": mesh_info,
        "bone_check": bone_check,
        "missing_needed_joints": [b for b, v in bone_check.items() if not v["in_joints"]],
        "root_nodes": root_nodes,
        "hierarchy_sample": hierarchy,
        "blocker": None,
    }
    if len(skins) == 0 or skinned_prims == 0:
        report["blocker"] = "NO_SKIN_OR_WEIGHTS"
    elif report["missing_needed_joints"]:
        report["blocker"] = "MISSING_KEY_BONES:" + ",".join(report["missing_needed_joints"])

    OUT.write_text(json.dumps(report, indent=2))
    print("GLB_INSPECT", json.dumps({
        "n_skins": len(skins),
        "skinned_prims": skinned_prims,
        "n_joints": skin_info[0]["n_joints"] if skin_info else 0,
        "missing": report["missing_needed_joints"],
        "blocker": report["blocker"],
        "meshes": [(m["name"], m["node_users"], m["skin_on_nodes"]) for m in mesh_info],
    }, indent=2))


if __name__ == "__main__":
    main()
