"""python apply_to_blend.py SRC.blend DST.blend  -- dress Xandra in the black lingerie set."""
import sys, os, runpy, bpy
src, dst = sys.argv[-2], sys.argv[-1]
bpy.ops.wm.open_mainfile(filepath=src)
ns = runpy.run_path(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build_outfit.py'))
parent = bpy.data.collections.get('Clothes')
ns['build'](hide=('Xandra_Top', 'Xandra_Shorts'), parent_coll=parent)
bpy.ops.wm.save_as_mainfile(filepath=dst, compress=True)
print('XMS_SAVED', dst)
