# Black lingerie outfit for Xandra

`build_outfit.py` generates a black satin corset (front/back lacing, lace trim,
off-shoulder lace sleeves), high-waist satin briefs with a lace garter belt,
six garter straps with hardware, and sheer black stockings with lace tops.
Garments are cut from Xandra's bind-pose body surface, skinned with weights
interpolated from the body (deform bones only) and parented to the `root`
armature, so every existing action/NLA strip drives them.

Body skin under opaque satin is hidden through the vertex group
`XMS_Lingerie_Hidden`; the existing `Clothes_Mask` modifiers are retargeted to it
(the original `Clothes_Hidden` group is kept). `Xandra_Top` / `Xandra_Shorts`
are hidden, not deleted.

Rebuild the dressed copy (Blender 4.5 LTS, or `pip install bpy==4.5.13` on Python 3.11):

```sh
python scripts/outfit/apply_to_blend.py assets/Xandra_Animations_look.blend assets/Xandra_Animations_lingerie.blend
# or: blender -b --python scripts/outfit/apply_to_blend.py -- SRC.blend DST.blend
```

Previews: `docs/outfit/`.

## Black Lace fitting (v01)

`assets/Xandra_BlackLace_Fitting_v01.blend` is an external fitting prototype
(collection `Xandra_BlackLace_Fitting_v01`, objects `BL_*`, skinned to `root`).
`blacklace_mask.py` added the skin mask under the opaque corset and briefs
(vertex group `BL_Hidden`, used by the re-enabled `Clothes_Mask` modifiers).

`assets/Xandra_BlackLace.glb` is exported from it with the same 8 clips, 52 head
morphs and 341-joint skin as `Xandra_Animated.glb`; open it in the viewer with
`viewer/animation.html?outfit=blacklace`.

```sh
blender -b SRC.blend --python scripts/outfit/blacklace_mask.py -- assets/Xandra_BlackLace_Fitting_v01.blend
blender -b assets/Xandra_BlackLace_Fitting_v01.blend --python scripts/outfit/export_blacklace_glb.py -- assets/Xandra_BlackLace.glb
```
