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
