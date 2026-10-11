"""Build the black satin/lace lingerie outfit on Xandra (armature 'root').

Run inside Blender with the character already loaded. All garments are generated
from the body's bind-pose surface, skinned with weights interpolated from the body,
and parented to the armature, so existing animation drives them.
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
from mathutils.interpolate import poly_3d_calc

ARM_NAME = 'root'
COLL_NAME = 'XMS_Outfit_BlackLingerie'
YC = -0.03            # torso centre (front is -Y)

# ---------------------------------------------------------------- helpers
def smoothstep(e0, e1, x):
    t = min(1.0, max(0.0, (x - e0) / (e1 - e0)))
    return t * t * (3 - 2 * t)


def theta(p):
    """Azimuth around the torso; 0 = centre front, +-pi = centre back."""
    return math.atan2(p.x, -(p.y - YC))


def thigh_frame(p):
    side = 1.0 if p.x >= 0 else -1.0
    xc = side * (0.081 + (0.901 - p.z) * (0.023 / 0.434))
    v = Vector((p.x - xc, p.y + 0.022))
    if v.length < 1e-6:
        return side, 0.0, 0.0, 0.0
    v.normalize()
    lateral = side * v.x
    front = -v.y
    phi = math.atan2(front, lateral)  # 0 lateral, +pi/2 front
    return side, lateral, front, phi


# ---------------------------------------------------------------- body proxy
def bind_coords(o):
    """Vertex positions with active shape keys but no armature/mask deformation."""
    states = [(m, m.show_viewport) for m in o.modifiers]
    for m, _ in states:
        m.show_viewport = False
    try:
        dg = bpy.context.evaluated_depsgraph_get()
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        cos = [v.co.copy() for v in me.vertices] if len(me.vertices) == len(o.data.vertices) else [v.co.copy() for v in o.data.vertices]
        ev.to_mesh_clear()
    finally:
        for m, st in states:
            m.show_viewport = st
    return cos


class Body:
    def __init__(self, objs, arm, no_arms=False):
        self.bm = bmesh.new()
        self.weights = []
        armw = self.bm.verts.layers.float.new('armw')
        pid = self.bm.verts.layers.int.new('pid')
        index = {}
        deform = {b.name for b in arm.data.bones if b.use_deform}
        for o in objs:
            mw = o.matrix_world
            me = o.data
            names = [g.name for g in o.vertex_groups]
            local = []
            bind = bind_coords(o)
            for v in me.vertices:
                co = mw @ bind[v.index]
                key = (round(co.x, 5), round(co.y, 5), round(co.z, 5))
                if key in index:
                    local.append(index[key]); continue
                bv = self.bm.verts.new(co)
                w = {names[g.group]: g.weight for g in v.groups if g.weight > 1e-4 and names[g.group] in deform}
                tot = sum(w.values()) or 1.0
                w = {k: x / tot for k, x in w.items()}
                bv[pid] = len(self.weights)
                bv[armw] = sum(x for k, x in w.items() if k.startswith(('upperarm', 'lowerarm', 'hand')))
                self.weights.append(w)
                index[key] = bv
                local.append(bv)
            for poly in me.polygons:
                vs = [local[i] for i in poly.vertices]
                if len(set(vs)) < 3:
                    continue
                try:
                    self.bm.faces.new(vs)
                except ValueError:
                    pass
        if no_arms:
            bmesh.ops.delete(self.bm, geom=[f for f in self.bm.faces if max(v[armw] for v in f.verts) > 0.35], context='FACES')
            bmesh.ops.delete(self.bm, geom=[v for v in self.bm.verts if not v.link_faces], context='VERTS')
            self.weights = [self.weights[v[pid]] for v in self.bm.verts]
        self.bm.verts.index_update()
        self.bm.verts.ensure_lookup_table()
        self.bm.faces.ensure_lookup_table()
        self.bm.normal_update()
        self.bvh = BVHTree.FromBMesh(self.bm)

    def sample(self, p):
        """Nearest surface point, interpolated smooth normal, barycentric verts."""
        loc, fn, fi, d = self.bvh.find_nearest(p)
        f = self.bm.faces[fi]
        cos = [v.co for v in f.verts]
        bw = poly_3d_calc(cos, loc)
        n = Vector()
        for v, w in zip(f.verts, bw):
            n += v.normal * w
        n.normalize()
        return loc, n, fn, f, bw

    def weights_at(self, p, strip_arms=False):
        loc, n, fn, f, bw = self.sample(p)
        out = {}
        for v, w in zip(f.verts, bw):
            for k, x in self.weights[v.index].items():
                if strip_arms and k.startswith(('upperarm', 'lowerarm', 'hand')):
                    continue
                out[k] = out.get(k, 0.0) + x * w
        top = sorted(out.items(), key=lambda kv: -kv[1])[:6]
        tot = sum(x for _, x in top) or 1.0
        return {k: x / tot for k, x in top if x / tot > 1e-3}

    def push_out(self, p, off):
        loc, n, fn, f, bw = self.sample(p)
        if fn.dot(p - loc) < 0 and (p - loc).length > 1e-6:
            return loc + n * off  # inside the body
        if (p - loc).dot(n) < off:
            return loc + n * off
        return p


# ---------------------------------------------------------------- shells
def cut_by_field(bm, fl):
    new = []
    for e in list(bm.edges):
        a, b = e.verts
        fa, fb = a[fl], b[fl]
        if (fa < 0) != (fb < 0):
            t = fa / (fa - fb)
            ne, nv = bmesh.utils.edge_split(e, a, t)
            nv[fl] = 0.0
            new.append(nv)
    if new:
        bmesh.ops.connect_verts(bm, verts=new)
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if v[fl] < -1e-9], context='VERTS')
    # drop slivers / loose bits
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')


def make_shell(body, field, offset, subdiv=2, smooth=4, margin=0.03, attrs=None, keep=None, inflate='nearest'):
    """field(co, armw) -> signed margin (m), >0 inside garment."""
    bm = body.bm.copy()
    armw = bm.verts.layers.float['armw']
    fl = bm.verts.layers.float.new('f')
    for v in bm.verts:
        v[fl] = field(v.co, v[armw])
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if max(v[fl] for v in f.verts) < -margin], context='FACES')
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    for _ in range(subdiv):
        bmesh.ops.subdivide_edges(bm, edges=list(bm.edges), cuts=1, use_grid_fill=True)
        bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4])
    for v in bm.verts:
        v[fl] = field(v.co, v[armw])
    cut_by_field(bm, fl)
    bmesh.ops.holes_fill(bm, edges=[e for e in bm.edges if e.is_boundary], sides=40)
    bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4])
    if keep:
        keep(bm)
    # attributes on the body-surface position
    data = {k: [] for k in (attrs or {})}
    bm.verts.index_update()
    for v in bm.verts:
        for k, fn in (attrs or {}).items():
            data[k].append(fn(v.co))
    # offset along interpolated body normal
    for v in bm.verts:
        loc, n, *_ = body.sample(v.co)
        v.co = loc + n * offset
    # relax (bridges clefts / softens nipples) while staying outside the body
    for _ in range(smooth):
        boundary = {v for v in bm.verts if v.is_boundary}
        newco = {}
        for v in bm.verts:
            nb = [e.other_vert(v).co for e in v.link_edges]
            if not nb:
                continue
            avg = sum(nb, Vector()) / len(nb)
            newco[v] = v.co.lerp(avg, 0.25 if v in boundary else 0.6)
        bm.normal_update()
        for v, c in newco.items():
            loc, n, *_ = body.sample(c)
            d = (c - loc).dot(n)
            if inflate == 'normal' and d > -0.004:
                if d < offset:
                    c = c + v.normal * (offset - d)
            else:
                c = body.push_out(c, offset)
            v.co = c
    bm.normal_update()
    return bm, data


def bm_to_object(bm, name, body, mat, coll, attrs=None, weights=True, rest=True):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    for p in me.polygons:
        p.use_smooth = True
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    me.materials.append(mat)
    if rest:
        a = me.attributes.new('rest', 'FLOAT_VECTOR', 'POINT')
        a.data.foreach_set('vector', [c for v in me.vertices for c in v.co])
    for k, vals in (attrs or {}).items():
        a = me.attributes.new(k, 'FLOAT', 'POINT')
        a.data.foreach_set('value', vals)
    if weights:
        skin(ob, body)
    return ob


def skin(ob, body):
    groups = {}
    strip = not ob.name.startswith('XMS_Sleeve')
    for v in ob.data.vertices:
        for k, w in body.weights_at(v.co, strip).items():
            g = groups.get(k) or ob.vertex_groups.new(name=k)
            groups[k] = g
            g.add([v.index], w, 'REPLACE')


def attach(ob, arm):
    ob.parent = arm
    ob.matrix_parent_inverse = arm.matrix_world.inverted()
    m = ob.modifiers.new('Armature', 'ARMATURE')
    m.object = arm


# ---------------------------------------------------------------- shapes
def corset_top(p):
    a = abs(theta(p))
    if a < 0.62:
        return 1.236 + 0.072 * math.sin(a / 0.62 * math.pi / 2)
    if a < 1.5:
        return 1.308 - 0.022 * (a - 0.62) / 0.88
    return 1.286 - 0.066 * ((a - 1.5) / (math.pi - 1.5))


def corset_bottom(p):
    t = theta(p)
    return 1.062 - 0.016 * math.exp(-(t / 0.16) ** 2)


def panty_top(p):
    return 1.046 + 0.006 * (1 - math.cos(theta(p))) / 2


STRAP_A = (0.80, 1.45, 2.30)
STRAP_DIP = (0.072, 0.036, 0.07)


def lace_bottom(p, dips=True):
    t = theta(p); a = abs(t)
    z = 0.986 - 0.012 * math.exp(-(t / 0.07) ** 2)
    if dips:
        for c, depth in zip(STRAP_A, STRAP_DIP):
            z -= depth * math.exp(-((a - c) / 0.17) ** 2)
    return z


def leg_line(p):
    side, lat, fr, phi = thigh_frame(p)
    return 0.866 + 0.08 * lat + 0.012 * fr


def stocking_top(p):
    side, lat, fr, phi = thigh_frame(p)
    return 0.664 + 0.008 * lat


def not_arm(armw):
    return 0.5 - armw


def torso_arc(p):
    return theta(p) * 0.14


# ---------------------------------------------------------------- materials
class NB:
    def __init__(self, mat):
        mat.use_nodes = True
        self.t = mat.node_tree
        self.t.nodes.clear()
        self.out = self.new('ShaderNodeOutputMaterial', (900, 0))
        self.bsdf = self.new('ShaderNodeBsdfPrincipled', (600, 0))
        self.link(self.bsdf.outputs[0], self.out.inputs[0])

    def new(self, typ, loc=(0, 0), **kw):
        n = self.t.nodes.new(typ)
        n.location = loc
        for k, v in kw.items():
            setattr(n, k, v)
        return n

    def link(self, a, b):
        self.t.links.new(a, b)

    def _in(self, sock, val):
        if isinstance(val, (int, float, tuple, list)):
            sock.default_value = val
        else:
            self.link(val, sock)

    def math(self, op, a, b=0.0, c=0.0, clamp=False):
        n = self.new('ShaderNodeMath', operation=op, use_clamp=clamp)
        self._in(n.inputs[0], a)
        if len(n.inputs) > 1: self._in(n.inputs[1], b)
        if len(n.inputs) > 2: self._in(n.inputs[2], c)
        return n.outputs[0]

    def smooth(self, e0, e1, x):
        n = self.new('ShaderNodeMapRange', interpolation_type='SMOOTHSTEP', clamp=True)
        self._in(n.inputs['Value'], x)
        n.inputs['From Min'].default_value = e0
        n.inputs['From Max'].default_value = e1
        return n.outputs['Result']

    def attr(self, name, out='Fac'):
        n = self.new('ShaderNodeAttribute', attribute_type='GEOMETRY', attribute_name=name)
        return n.outputs[out]

    def xyz(self):
        s = self.new('ShaderNodeSeparateXYZ')
        self.link(self.attr('rest', 'Vector'), s.inputs[0])
        return s.outputs[0], s.outputs[1], s.outputs[2]

    def set(self, name, val):
        self._in(self.bsdf.inputs[name], val)


def common(mat):
    mat.surface_render_method = 'DITHERED'
    if hasattr(mat, 'use_transparent_shadow'):
        mat.use_transparent_shadow = True


def mat_satin(name, seams=False):
    m = bpy.data.materials.new(name); nb = NB(m); common(m)
    nb.set('Base Color', (0.009, 0.009, 0.010, 1))
    nb.set('Roughness', 0.36)
    nb.set('Specular IOR Level', 0.38)
    nb.set('Anisotropic', 0.7)
    nb.set('Sheen Weight', 0.45)
    nb.set('Sheen Roughness', 0.3)
    nb.set('Sheen Tint', (0.45, 0.45, 0.5, 1))
    height = None
    if seams:
        x, y, z = nb.xyz()
        th = nb.math('ARCTAN2', x, nb.math('MULTIPLY', nb.math('ADD', y, -YC), -1.0))
        a = nb.math('ABSOLUTE', th)
        d = 10.0
        for c in (0.40, 1.02, 1.58, 2.25, 2.86):
            d = nb.math('MINIMUM', d, nb.math('ABSOLUTE', nb.math('SUBTRACT', a, c)))
        dv = nb.math('MULTIPLY', d, 0.14)
        # underbust/cup seam, front only
        ax = nb.math('ABSOLUTE', x)
        dx = nb.math('SUBTRACT', ax, 0.087)
        k = nb.math('ADD', 5.5, nb.math('MULTIPLY', nb.math('GREATER_THAN', dx, 0.0), 6.0))
        zub = nb.math('ADD', 1.192, nb.math('MULTIPLY', k, nb.math('MULTIPLY', dx, dx)))
        front = nb.math('LESS_THAN', a, 1.35)
        below = nb.math('LESS_THAN', z, zub)
        # front vertical seams stop at the cup seam
        vmask = nb.math('MAXIMUM', below, nb.math('GREATER_THAN', a, 1.35))
        vr = nb.math('MULTIPLY', nb.math('SUBTRACT', 1.0, nb.smooth(0.0006, 0.0028, dv)), vmask)
        du = nb.math('ABSOLUTE', nb.math('SUBTRACT', z, zub))
        ur = nb.math('MULTIPLY', nb.math('SUBTRACT', 1.0, nb.smooth(0.0006, 0.0028, du)), front)
        height = nb.math('MAXIMUM', vr, ur)
    noise = nb.new('ShaderNodeTexNoise', (-300, -400))
    noise.inputs['Scale'].default_value = 900.0
    noise.inputs['Detail'].default_value = 2.0
    nb.link(nb.attr('rest', 'Vector'), noise.inputs['Vector'])
    bump = nb.new('ShaderNodeBump', (300, -300))
    bump.inputs['Strength'].default_value = 0.6
    bump.inputs['Distance'].default_value = 0.0015
    h = nb.math('MULTIPLY', noise.outputs['Fac'], 0.01)
    if height is not None:
        h = nb.math('ADD', h, height)
    nb.link(h, bump.inputs['Height'])
    nb.link(bump.outputs['Normal'], nb.bsdf.inputs['Normal'])
    return m


def lace_pattern(nb, scale=1.0, density=1.0):
    rest = nb.attr('rest', 'Vector')
    vor = nb.new('ShaderNodeTexVoronoi', feature='DISTANCE_TO_EDGE')
    vor.inputs['Scale'].default_value = 230.0 * scale
    nb.link(rest, vor.inputs['Vector'])
    net = nb.math('SUBTRACT', 1.0, nb.smooth(0.03, 0.10, vor.outputs['Distance']))
    noi = nb.new('ShaderNodeTexNoise')
    noi.inputs['Scale'].default_value = 32.0 * scale
    noi.inputs['Detail'].default_value = 3.0
    noi.inputs['Roughness'].default_value = 0.55
    noi.inputs['Distortion'].default_value = 0.5
    nb.link(rest, noi.inputs['Vector'])
    f = noi.outputs['Fac']
    petal = nb.smooth(0.50, 0.53, f)
    vein = nb.new('ShaderNodeTexVoronoi', feature='DISTANCE_TO_EDGE')
    vein.inputs['Scale'].default_value = 70.0 * scale
    nb.link(rest, vein.inputs['Vector'])
    veins = nb.math('SUBTRACT', 1.0, nb.smooth(0.02, 0.05, vein.outputs['Distance']))
    petal_open = nb.math('MULTIPLY', petal, nb.math('SUBTRACT', 1.0, nb.math('MULTIPLY', veins, 0.8)))
    outline = nb.math('SUBTRACT', 1.0, nb.smooth(0.004, 0.014, nb.math('ABSOLUTE', nb.math('SUBTRACT', f, 0.50))))
    pat = nb.math('MAXIMUM', nb.math('MULTIPLY', net, 0.85), nb.math('MULTIPLY', petal_open, 0.97))
    pat = nb.math('MAXIMUM', pat, outline)
    pat = nb.math('MAXIMUM', pat, 0.10 * density)
    return pat


def mat_lace(name, scallop=0.006, period=0.017, backing=0.0):
    m = bpy.data.materials.new(name); nb = NB(m); common(m)
    nb.set('Base Color', (0.006, 0.006, 0.007, 1))
    nb.set('Roughness', 0.55)
    nb.set('Sheen Weight', 0.4)
    nb.set('Sheen Tint', (0.5, 0.5, 0.5, 1))
    pat = lace_pattern(nb)
    d = nb.attr('lace_d'); s = nb.attr('lace_s')
    wave = nb.math('ABSOLUTE', nb.math('SINE', nb.math('MULTIPLY', s, math.pi / period)))
    edge = nb.math('MULTIPLY', nb.math('SUBTRACT', 1.0, nb.math('POWER', wave, 0.7)), scallop)
    inside = nb.smooth(0.0, 0.0006, nb.math('SUBTRACT', d, edge))
    border = nb.math('SUBTRACT', 1.0, nb.smooth(0.0025, 0.0045, nb.math('SUBTRACT', d, edge)))
    # small picot loops along the scallop
    pico = nb.math('MULTIPLY', nb.math('SUBTRACT', 1.0, nb.smooth(0.0, 0.0012, nb.math('ABSOLUTE', nb.math('SUBTRACT', d, nb.math('SUBTRACT', edge, 0.0012))))),
                   nb.math('GREATER_THAN', nb.math('SINE', nb.math('MULTIPLY', s, 2 * math.pi / 0.0035)), 0.2))
    a = nb.math('MAXIMUM', pat, nb.math('MULTIPLY', border, 0.96))
    if backing:
        a = nb.math('MAXIMUM', a, backing)
    a = nb.math('MAXIMUM', nb.math('MULTIPLY', a, inside), nb.math('MULTIPLY', pico, 0.9))
    nb.set('Alpha', a)
    bump = nb.new('ShaderNodeBump'); bump.inputs['Strength'].default_value = 0.5; bump.inputs['Distance'].default_value = 0.0008
    nb.link(pat, bump.inputs['Height']); nb.link(bump.outputs['Normal'], nb.bsdf.inputs['Normal'])
    return m


def mat_nylon(name):
    m = bpy.data.materials.new(name); nb = NB(m); common(m)
    nb.set('Base Color', (0.008, 0.008, 0.009, 1))
    nb.set('Roughness', 0.42)
    nb.set('Specular IOR Level', 0.45)
    nb.set('Sheen Weight', 0.6)
    nb.set('Sheen Roughness', 0.4)
    nb.set('Sheen Tint', (0.35, 0.35, 0.38, 1))
    lw = nb.new('ShaderNodeLayerWeight'); lw.inputs['Blend'].default_value = 0.45
    a = nb.new('ShaderNodeMapRange', clamp=True)
    nb.link(lw.outputs['Facing'], a.inputs['Value'])
    a.inputs['To Min'].default_value = 0.42
    a.inputs['To Max'].default_value = 0.94
    # denser welt just under the lace band, reinforced toe
    x, y, z = nb.xyz()
    welt = nb.smooth(0.585, 0.60, z)
    toe = nb.math('SUBTRACT', 1.0, nb.smooth(0.03, 0.05, z))
    al = nb.math('MAXIMUM', a.outputs['Result'], nb.math('MULTIPLY', nb.math('MAXIMUM', welt, toe), 0.9))
    nb.set('Alpha', al)
    return m


def mat_metal(name):
    m = bpy.data.materials.new(name); nb = NB(m)
    nb.set('Base Color', (0.03, 0.03, 0.035, 1)); nb.set('Metallic', 1.0); nb.set('Roughness', 0.3)
    return m


# ---------------------------------------------------------------- small geometry
def frame_from(n, up_hint=Vector((0, 0, 1))):
    n = n.normalized()
    u = (up_hint - n * up_hint.dot(n)).normalized()
    s = u.cross(n)
    return s, u, n


def add_torus(bm, center, axis_n, R, r, seg=16, rseg=8, squash=1.0):
    s, u, n = frame_from(axis_n)
    rings = []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        d = s * math.cos(a) + u * math.sin(a)
        ring = []
        for j in range(rseg):
            b = 2 * math.pi * j / rseg
            p = center + d * (R + r * math.cos(b)) + n * (r * math.sin(b) * squash)
            ring.append(bm.verts.new(p))
        rings.append(ring)
    for i in range(seg):
        for j in range(rseg):
            a, b = rings[i], rings[(i + 1) % seg]
            bm.faces.new((a[j], b[j], b[(j + 1) % rseg], a[(j + 1) % rseg]))


def add_box(bm, center, s, u, n, ws, wu, wn):
    vs = []
    for i in (-1, 1):
        for j in (-1, 1):
            for k in (-1, 1):
                vs.append(bm.verts.new(center + s * (i * ws / 2) + u * (j * wu / 2) + n * (k * wn / 2)))
    idx = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    for f in idx:
        bm.faces.new([vs[i] for i in f])


def add_tube(bm, pts, r, seg=8, close=False):
    rings = []
    for i, p in enumerate(pts):
        a = pts[max(i - 1, 0)]; b = pts[min(i + 1, len(pts) - 1)]
        t = (b - a).normalized()
        hint = Vector((0, 0, 1)) if abs(t.z) < 0.9 else Vector((1, 0, 0))
        s = t.cross(hint).normalized(); u = s.cross(t)
        rings.append([bm.verts.new(p + (s * math.cos(2 * math.pi * j / seg) + u * math.sin(2 * math.pi * j / seg)) * r) for j in range(seg)])
    n = len(rings)
    for i in range(n if close else n - 1):
        a, b = rings[i], rings[(i + 1) % n]
        for j in range(seg):
            bm.faces.new((a[j], b[j], b[(j + 1) % seg], a[(j + 1) % seg]))


def add_ribbon(bm, pts, normals, width):
    rows = []
    for i, p in enumerate(pts):
        a = pts[max(i - 1, 0)]; b = pts[min(i + 1, len(pts) - 1)]
        t = (b - a).normalized(); s = t.cross(normals[i]).normalized()
        rows.append((bm.verts.new(p - s * width / 2), bm.verts.new(p + s * width / 2)))
    for i in range(len(rows) - 1):
        bm.faces.new((rows[i][0], rows[i][1], rows[i + 1][1], rows[i + 1][0]))


def add_bow(bm, p, n, size, up=Vector((0, 0, 1))):
    s, u, n = frame_from(n, up)
    for side in (-1, 1):
        # loop: flattened ring tilted slightly upward
        c = p + s * (side * size * 0.33) + u * (size * 0.04)
        ax = (n * 0.9 + u * 0.15 * side).normalized()
        R = size * 0.30
        seg = 18
        loop = []
        for i in range(seg):
            a = 2 * math.pi * i / seg
            q = c + s * (math.cos(a) * R) + u * (math.sin(a) * R * 0.55)
            loop.append(q + n * (size * 0.06 * (1 - abs(math.cos(a)))))
        add_tube(bm, loop, size * 0.05, seg=6, close=True)
        tail = [p + s * (side * size * (0.05 + 0.18 * t)) - u * (size * (0.08 + 0.55 * t)) + n * (size * 0.04) for t in (0, 0.33, 0.66, 1.0)]
        add_ribbon(bm, tail, [n] * 4, size * 0.16)
    add_box(bm, p + n * (size * 0.05), s, u, n, size * 0.18, size * 0.2, size * 0.12)


# ---------------------------------------------------------------- build
def build(hide=(), parent_coll=None):
    arm = bpy.data.objects[ARM_NAME]
    body_objs = [o for o in bpy.data.objects if o.type == 'MESH' and o.name.startswith(('F4_Torso', 'F4_Legs', 'F4_Feet'))
                 and 'Censored' not in o.name and not o.hide_render]
    # hide any previous outfit / underwear
    old = bpy.data.collections.get(COLL_NAME)
    if old:
        for o in list(old.objects):
            bpy.data.objects.remove(o)
        bpy.data.collections.remove(old)
    for o in bpy.data.objects:
        if o.type == 'MESH' and (o.name in hide or 'underwear' in o.name.lower()):
            o.hide_render = True; o.hide_set(True)
    coll = bpy.data.collections.new(COLL_NAME)
    (parent_coll or bpy.context.scene.collection).children.link(coll)
    full = Body(body_objs, arm)
    body = Body(body_objs, arm, no_arms=True)

    satin = mat_satin('XMS_Satin_Black')
    satin_c = mat_satin('XMS_Satin_Black_Corset', seams=True)
    lace = mat_lace('XMS_Lace_Black', backing=0.55)
    lace_top = mat_lace('XMS_Lace_Black_Stocking', backing=0.62)
    nylon = mat_nylon('XMS_Nylon_Black')
    metal = mat_metal('XMS_Metal_Black')
    made = []

    def finish(ob, solid=0.0):
        if solid:
            m = ob.modifiers.new('Solidify', 'SOLIDIFY'); m.thickness = solid; m.offset = 1.0; m.use_rim = True
        attach(ob, arm)
        made.append(ob)
        return ob

    # --- corset
    def f_corset(p, aw):
        return min(p.z - corset_bottom(p), corset_top(p) - p.z, not_arm(aw) * 0.1, 0.19 - abs(p.x))
    bm, _ = make_shell(body, f_corset, 0.0045, smooth=60, inflate='normal')
    corset = finish(bm_to_object(bm, 'XMS_Corset', body, satin_c, coll), 0.0018)
    piping(bm, 'XMS_Corset_Binding', body, satin, coll, arm, made, out=0.0025)
    bm.free()

    # --- neckline lace trim
    def f_trim(p, aw):
        zt = corset_top(p)
        return min(p.z - (zt - 0.012), (zt + 0.034) - p.z, not_arm(aw) * 0.1, 0.19 - abs(p.x))
    bm, at = make_shell(body, f_trim, 0.0068, smooth=40, inflate='normal',
                        attrs={'lace_d': lambda p: corset_top(p) + 0.03 - p.z, 'lace_s': torso_arc})
    finish(bm_to_object(bm, 'XMS_Corset_LaceTrim', body, lace, coll, at)); bm.free()

    # --- off-shoulder lace sleeves
    for side in ('l', 'r'):
        b0 = arm.matrix_world @ arm.data.bones['upperarm_' + side].head_local
        b1 = arm.matrix_world @ arm.data.bones['lowerarm_' + side].head_local
        ax = (b1 - b0).normalized()
        sg = 1 if side == 'l' else -1

        def f_sleeve(p, aw, b0=b0, ax=ax, sg=sg):
            t = (p - b0).dot(ax)
            return min(t - 0.05, 0.112 - t, (aw - 0.5) * 0.1, sg * p.x)

        def s_sleeve(p, b0=b0, ax=ax):
            r = (p - b0) - ax * (p - b0).dot(ax)
            s, u, n = frame_from(ax, Vector((0, -1, 0)))
            return math.atan2(r.dot(s), r.dot(u)) * 0.045

        def d_sleeve(p, b0=b0, ax=ax):
            t = (p - b0).dot(ax)
            return min(t - 0.05, 0.112 - t)
        bm, at = make_shell(full, f_sleeve, 0.003, smooth=3, attrs={'lace_d': d_sleeve, 'lace_s': s_sleeve})
        finish(bm_to_object(bm, 'XMS_Sleeve_' + side.upper(), full, lace, coll, at)); bm.free()

    # --- panties (satin: waistband + body below lace panel)
    def f_panty(p, aw):
        band = min(panty_top(p) - p.z, p.z - (panty_top(p) - 0.009))
        lower = min(lace_bottom(p, dips=False) + 0.004 - p.z, p.z - leg_line(p))
        return min(max(band, lower), not_arm(aw) * 0.1, 0.2 - abs(p.x))
    bm, _ = make_shell(body, f_panty, 0.0028, subdiv=1, smooth=45)
    finish(bm_to_object(bm, 'XMS_Panties', body, satin, coll), 0.0009)
    piping(bm, 'XMS_Panties_Binding', body, satin, coll, arm, made, out=0.0022, r=0.0012)
    bm.free()

    # --- lace panel / garter belt
    def f_lpanel(p, aw):
        return min(panty_top(p) - 0.006 - p.z, p.z - (lace_bottom(p) - 0.008), p.z - leg_line(p) + 0.03, not_arm(aw) * 0.1, 0.2 - abs(p.x))
    bm, at = make_shell(body, f_lpanel, 0.0050, smooth=45,
                        attrs={'lace_d': lambda p: p.z - lace_bottom(p), 'lace_s': torso_arc})
    lpanel = finish(bm_to_object(bm, 'XMS_GarterBelt_Lace', body, lace, coll, at)); bm.free()

    # --- stockings (nylon) + lace tops
    def f_stock(p, aw):
        return 0.607 - p.z
    bm, _ = make_shell(body, f_stock, 0.0011, subdiv=1, smooth=2)
    finish(bm_to_object(bm, 'XMS_Stockings', body, nylon, coll)); bm.free()

    def f_stop(p, aw):
        return min(stocking_top(p) - p.z, p.z - 0.598)
    bm, at = make_shell(body, f_stop, 0.0019, smooth=2,
                        attrs={'lace_d': lambda p: min(stocking_top(p) - p.z, p.z - 0.598 + 0.004),
                               'lace_s': lambda p: thigh_frame(p)[3] * 0.075})
    finish(bm_to_object(bm, 'XMS_Stocking_LaceTops', body, lace_top, coll, at)); bm.free()

    # --- lacing, eyelets, bows, garter straps
    corset_bvh = surface_bvh(corset)
    lacing(corset_bvh, body, coll, arm, made, satin, metal, front=True)
    lacing(corset_bvh, body, coll, arm, made, satin, metal, front=False)
    bows = bmesh.new()
    p, n = hit(corset_bvh, 0.0, 1.054, front=False)
    if p: add_bow(bows, p + n * 0.003, n, 0.05)
    lp_bvh = surface_bvh(lpanel)
    p, n = hit(lp_bvh, 0.0, 1.032, front=True)
    if p: add_bow(bows, p + n * 0.002, n, 0.018)
    finish(bm_to_object(bows, 'XMS_Bows', body, satin, coll, rest=True)); bows.free()
    straps(body, lp_bvh, coll, arm, made, satin, metal)

    for o in made:
        o.data.update()
    mask_body(body_objs + [o for o in bpy.data.objects if o.name.startswith('F4_Head')], f_corset, f_panty)
    print('XMS_OUTFIT', len(made), 'objects')
    return made


HIDDEN_GROUP = 'XMS_Lingerie_Hidden'


def mask_body(objs, *fields):
    """Hide skin well inside opaque satin; retarget existing clothing masks."""
    for o in objs:
        g = o.vertex_groups.get(HIDDEN_GROUP) or o.vertex_groups.new(name=HIDDEN_GROUP)
        g.remove(list(range(len(o.data.vertices))))
        if not o.name.startswith('F4_Head'):
            bind = bind_coords(o)
            names = [x.name for x in o.vertex_groups]
            hid = []
            for v in o.data.vertices:
                aw = sum(x.weight for x in v.groups if names[x.group].startswith(('upperarm', 'lowerarm', 'hand')))
                p = o.matrix_world @ bind[v.index]
                if max(f(p, aw) for f in fields) > 0.018:
                    hid.append(v.index)
            if hid:
                g.add(hid, 1.0, 'REPLACE')
        for m in o.modifiers:
            if m.type == 'MASK' and m.mode == 'VERTEX_GROUP':
                m.vertex_group = HIDDEN_GROUP
                m.invert_vertex_group = True


def surface_bvh(ob):
    bm = bmesh.new(); bm.from_mesh(ob.data)  # garment data is authored in world space
    return BVHTree.FromBMesh(bm), bm


def hit(sb, x, z, front=True):
    bvh, _ = sb
    y0 = -1.0 if front else 1.0
    d = Vector((0, 1 if front else -1, 0))
    loc, n, i, dist = bvh.ray_cast(Vector((x, y0, z)), d)
    if loc is None:
        return None, None
    if n.dot(d) > 0:
        n = -n
    return loc, n


def piping(bm, name, body, mat, coll, arm, made, out=0.002, r=0.0016):
    edges = [e for e in bm.edges if e.is_boundary]
    adj = {}
    for e in edges:
        a, b = e.verts
        adj.setdefault(a, []).append(b); adj.setdefault(b, []).append(a)
    seen = set(); out_bm = bmesh.new()
    for start in list(adj):
        if start in seen:
            continue
        loop = [start]; seen.add(start); prev = None; cur = start
        while True:
            nxt = [v for v in adj[cur] if v is not prev and v not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]; loop.append(cur); seen.add(cur)
        if len(loop) < 8:
            continue
        pts = []
        for v in loop:
            loc, n, *_ = body.sample(v.co)
            pts.append(v.co + v.normal * (out - 0.0016) + n * 0.0008)
        # light smoothing of the path
        for _ in range(3):
            pts = [pts[i] if i in (0,) else (pts[i - 1] + pts[i] * 2 + pts[(i + 1) % len(pts)]) / 4 for i in range(len(pts))]
        closed = (loop[-1] in adj[loop[0]])
        add_tube(out_bm, pts, r, seg=8, close=closed)
    ob = bm_to_object(out_bm, name, body, mat, coll)
    out_bm.free()
    attach(ob, arm); made.append(ob)


def lacing(sb, body, coll, arm, made, cord_mat, metal, front=True):
    zs = [1.084 + i * 0.02 for i in range(8)] if front else [1.078 + i * 0.019 for i in range(7)]
    dx = 0.011 if front else 0.013
    holes = []
    eb = bmesh.new()
    for z in zs:
        row = []
        for sx in (-1, 1):
            p, n = hit(sb, sx * dx, z, front)
            if p is None:
                continue
            p = p + n * 0.0019
            add_torus(eb, p, n, 0.0021, 0.0007, seg=12, rseg=6)
            row.append((p, n))
        if len(row) == 2:
            holes.append(row)
    ob = bm_to_object(eb, 'XMS_Eyelets_' + ('Front' if front else 'Back'), body, metal, coll)
    attach(ob, arm); made.append(ob); eb.free()
    cb = bmesh.new()
    for start in (0, 1):
        pts = []
        for i, row in enumerate(holes):
            p, n = row[(i + start) % 2]
            pts.append(p + n * 0.0012)
            if i + 1 < len(holes):
                q, m = holes[i + 1][(i + 1 + start) % 2]
                mid = (p + q) / 2 + (n + m).normalized() * (0.0026 + 0.0008 * start)
                pts.append(mid)
        add_tube(cb, smooth_path(pts, 2), 0.001, seg=6)
    ob = bm_to_object(cb, 'XMS_Lacing_' + ('Front' if front else 'Back'), body, cord_mat, coll)
    attach(ob, arm); made.append(ob); cb.free()


def smooth_path(pts, it):
    for _ in range(it):
        out = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            out += [a * 0.75 + b * 0.25, a * 0.25 + b * 0.75]
        out.append(pts[-1]); pts = out
    return pts


def straps(body, lp_sb, coll, arm, made, satin, metal):
    sbm = bmesh.new(); hw = bmesh.new()
    lp_bvh, _ = lp_sb
    for side in (1, -1):
        for a, dirn in zip(STRAP_A, ((0.15, -1.0), (1.0, -0.35), (0.4, 1.0))):
            # start: on the lace tab
            th = side * a
            zs = 0.986 - STRAP_DIP[STRAP_A.index(a)] + 0.006
            d = Vector((math.sin(th), -math.cos(th), 0))
            o = Vector((0, YC, zs)) + d * 0.4
            loc, n, i, dist = lp_bvh.ray_cast(o, -d)
            if loc is None:
                loc, n, *_ = body.sample(o)
            start = loc + n.normalized() * 0.0015
            # end: on the stocking lace top
            ze = 0.640
            xc = side * (0.081 + (0.901 - ze) * (0.023 / 0.434))
            dd = Vector((side * dirn[0], dirn[1], 0)).normalized()
            c = Vector((xc, -0.022, ze))
            loc2, _, _, _ = body.bvh.ray_cast(c + dd * 0.3, -dd)
            loc2, n2, *_ = body.sample(loc2 if loc2 else c + dd * 0.07)
            end = loc2 + n2 * 0.0045
            pts = [start.lerp(end, t / 40) for t in range(41)]
            for _ in range(6):
                pts = [pts[0]] + [body.push_out((pts[i - 1] + pts[i] * 2 + pts[i + 1]) / 4, 0.0045) for i in range(1, 40)] + [pts[-1]]
            nrm = [body.sample(p)[1] for p in pts]
            add_ribbon(sbm, pts, nrm, 0.011)
            # adjuster slider and clip
            for t, (ws, wu) in ((0.35, (0.016, 0.006)), (1.0, (0.015, 0.012))):
                i = min(40, int(t * 40)); p = pts[i]; nn = nrm[i]
                tg = (pts[min(40, i + 1)] - pts[max(0, i - 1)]).normalized()
                s = tg.cross(nn).normalized(); u = nn.cross(s)
                add_box(hw, p + nn * 0.0012 - u * (0.004 if t == 1.0 else 0), s, u, nn, ws, wu, 0.0022)
    ob = bm_to_object(sbm, 'XMS_GarterStraps', body, satin, coll)
    m = ob.modifiers.new('Solidify', 'SOLIDIFY'); m.thickness = 0.0012; m.offset = 0
    attach(ob, arm); made.append(ob)
    ob = bm_to_object(hw, 'XMS_GarterHardware', body, metal, coll)
    attach(ob, arm); made.append(ob)
    sbm.free(); hw.free()


if __name__ == '__main__':
    build()
