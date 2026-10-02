"""Rebuild collision hull inputs from Xandra's rest skin; numpy + MuJoCo only.

Run from any directory with .venv/bin/python scripts/build_collision_surfaces.py.
The frozen pre-change model supplies inertias, joints, actuator order and limits.
Vertex ownership sums skin weights by physical segment. MuJoCo constructs convex
hulls (at most 64 hull vertices) at load time. These are rigid approximations,
not deformable skin contacts. Fingers remain part of the hand collision hull.
"""
import json
import struct
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
from bridge_full.mapping_full import BODY_TO_BONE, DERIVED_BONE_SOURCES


def skin_points():
    raw = (ROOT / 'assets/Xandra.glb').read_bytes()
    size = struct.unpack_from('<I', raw, 12)[0]
    gltf = json.loads(raw[20:20 + size])
    buffer = raw[28 + size:]

    def accessor(index):
        a = gltf['accessors'][index]
        v = gltf['bufferViews'][a['bufferView']]
        dtype = np.dtype({5126: '<f4', 5123: '<u2', 5121: 'u1', 5125: '<u4'}[a['componentType']])
        width = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}[a['type']]
        return np.ndarray((a['count'], width), dtype=dtype, buffer=buffer,
                          offset=v.get('byteOffset', 0) + a.get('byteOffset', 0),
                          strides=(v.get('byteStride', dtype.itemsize * width), dtype.itemsize))

    parents = {child: i for i, node in enumerate(gltf['nodes']) for child in node.get('children', [])}
    lookup = {bone: body for body, bone in BODY_TO_BONE.items()}
    lookup.update({bone: source if isinstance(source, str) else source[1]
                   for bone, source in DERIVED_BONE_SOURCES.items()})

    def owner(index):
        while index is not None:
            name = gltf['nodes'][index].get('name')
            if name in lookup:
                return lookup[name]
            index = parents.get(index)
        return None

    points = {}
    # These four meshes export meter-valued bind-space vertices. Exclude hair,
    # earrings and underwear. Retain the existing head sphere.
    for node in gltf['nodes']:
        if node.get('name') not in ('F4_Torso', 'F4_Legs_Nude.001', 'F4_Hands', 'F4_Feet'):
            continue
        owners = np.array([owner(i) for i in gltf['skins'][node['skin']]['joints']])
        names = sorted(set(owners) - {None})
        for primitive in gltf['meshes'][node['mesh']]['primitives']:
            a = primitive['attributes']
            vertices = accessor(a['POSITION']).astype(float)[:, [0, 2, 1]]
            vertices[:, 1] *= -1  # glTF Y-up -> physical Z-up
            groups = owners[accessor(a['JOINTS_0'])]
            weights = accessor(a['WEIGHTS_0'])
            sums = np.stack([np.sum(weights * (groups == name), axis=1) for name in names], axis=1)
            assigned = np.array(names)[np.argmax(sums, axis=1)]
            for name in names:
                points.setdefault(name, []).extend(vertices[assigned == name])
    return {name: np.unique(np.round(values, 6), axis=0)
            for name, values in points.items() if len(values)}


def numbers(values):
    return ' '.join(format(float(v), '.12g') for v in np.asarray(values).flat)


def main():
    baseline = ROOT / 'reports/collision/model_before.xml'
    model = mujoco.MjModel.from_xml_path(str(baseline))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    tree = ET.parse(baseline)
    root = tree.getroot()
    root.find('compiler').set('inertiafromgeom', 'auto')
    # Firm, damped contacts at the existing 2 ms timestep; no timing change.
    root.find('default/geom').set('solref', '0.004 1')
    root.find('default/geom').set('solimp', '0.95 0.99 0.001')
    asset, contacts = root.find('asset'), root.find('contact')
    contacts.clear()  # Remove broad body exclusions, including arm/torso pairs.
    directory = ROOT / 'assets/collision'
    directory.mkdir(exist_ok=True)
    points = skin_points()
    counts = {}

    def add_mesh(name, vertices):
        (directory / f'{name}.obj').write_text(
            '# Rest-skin points; MuJoCo computes the convex hull.\n' +
            '\n'.join('v ' + numbers(v) for v in vertices) + '\n')
        ET.SubElement(asset, 'mesh', name=name, file=f'../../assets/collision/{name}.obj', maxhullvert='64')
        counts[name] = len(vertices)

    for body in root.findall('.//body'):
        name = body.get('name')
        bid = model.body(name).id
        # Changing collision surfaces must not silently change mass properties.
        ET.SubElement(body, 'inertial', pos=numbers(model.body_ipos[bid]),
                      quat=numbers(model.body_iquat[bid]), mass=numbers([model.body_mass[bid]]),
                      diaginertia=numbers(model.body_inertia[bid]))
        if name not in points:
            continue
        geom = body.find('geom')
        vertices = points[name] - data.xpos[bid]
        add_mesh(name + '_surface', vertices)
        attributes = dict(name=geom.get('name'), type='mesh', mesh=name + '_surface',
                          rgba=geom.get('rgba', '0.6 0.6 0.6 1'), margin='0.006')
        geom.attrib.clear()
        geom.attrib.update(attributes)
        if name.startswith(('upperarm', 'lowerarm', 'thigh', 'calf', 'hand', 'foot')):
            # Parent/child overlap is needed at the shared joint seam, not along
            # the entire limb. Explicit pairs enable contact away from the seam.
            axis = model.geom(name + '_geom').pos.copy()
            axis /= np.linalg.norm(axis)
            distance = vertices @ axis
            distal = vertices[distance > max(.045, float(distance.max()) * .25)]
            add_mesh(name + '_distal', distal)
            ET.SubElement(body, 'geom', name=name + '_joint_guard', type='mesh',
                          mesh=name + '_distal', contype='0', conaffinity='0',
                          group='3', rgba='0.9 0.5 0.1 0.3')
            parent = model.body(int(model.body_parentid[bid])).name
            ET.SubElement(contacts, 'pair', geom1=name + '_joint_guard', geom2=parent + '_geom',
                          condim='3', friction='0.8 0.8 0.1 0.1 0.1', solref='0.004 1',
                          solimp='0.95 0.99 0.001', margin='0.006')
            if name.startswith('thigh'):
                # The abdomen and proximal thigh share a deforming hip seam
                # despite being uncle/nephew in the rigid hierarchy. Keep the
                # distal thigh contact instead of suppressing this whole pair.
                ET.SubElement(contacts, 'exclude', body1='torso_lower', body2=name)
                ET.SubElement(contacts, 'pair', geom1=name + '_joint_guard', geom2='torso_lower_geom',
                              condim='3', friction='0.8 0.8 0.1 0.1 0.1', solref='0.004 1',
                              solimp='0.95 0.99 0.001', margin='0.006')
    ET.indent(tree, space='  ')
    destination = ROOT / 'physics/model/humanoid_alpha.xml'
    destination.write_text('<!-- Collision hulls from Xandra rest skin. Rebuild with\n'
                           'scripts/build_collision_surfaces.py. Explicit inertias preserve\n'
                           'baseline mass properties; joint/actuator definitions unchanged. -->\n' +
                           ET.tostring(root, encoding='unicode') + '\n')
    updated = mujoco.MjModel.from_xml_path(str(destination))
    for field in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat',
                  'actuator_gear', 'actuator_ctrlrange', 'jnt_range', 'qpos0'):
        np.testing.assert_allclose(getattr(updated, field), getattr(model, field), rtol=1e-10, atol=1e-12)
    (ROOT / 'reports/collision/surface_vertex_counts.json').write_text(json.dumps(counts, indent=2) + '\n')
    print(f'Wrote {len(counts)} hull inputs; mass properties and actuator definitions preserved.')


if __name__ == '__main__':
    main()
