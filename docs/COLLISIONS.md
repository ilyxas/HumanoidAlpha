# Body collisions

Restart the running Python runtime and reload the viewer after this update.
Use the existing `stop-project.command`, then the usual launcher.

## Behavior

- Dynamic mode and `start-actuator-runtime`: limbs, hands, feet and torso now
  use convex collision envelopes extracted from the Xandra rest mesh. Head
  remains the existing sphere. Contacts exert physical reaction forces.
- Position mode: a proposed edit is checked along its whole interpolated path.
  If it crosses a collision surface, the edit is rejected and the previous pose
  remains. The lab shows `Movement blocked by contact`; sliders return to the
  actual value. Other joints and simulation time remain fixed.
- Position checks preserve approximately 6 mm of self-contact clearance, matching
  the dynamic contact margin. This avoids preloading the contact spring on Resume.
  The floor allows 2 mm numerical tolerance because the reference feet extend
  1.61 mm below zero in the exported skin.
- Visual clavicles follow the torso, matching their physical weld. Arm rotations
  no longer move the visual shoulder anchor away from the physical anchor.

## Physical model

The generated assets are `assets/collision/*.obj`: rest-skin point sets from
F4_Torso, F4_Legs_Nude.001, F4_Hands and F4_Feet. Vertex ownership uses summed skin
weights per physical segment; MuJoCo computes convex hulls with at most 64 hull
vertices per mesh. Hair, earrings and clothing are not collision shapes.

All broad arm/torso and other nonadjacent exclusions were removed. Adjacent joint
seams still need overlap: each limb has an explicit distal guard that collides
with its parent, leaving only the proximal joint seam free. The abdomen/proximal
thigh seam uses the same approach: two explicit body exclusions are replaced by
actual distal thigh/abdomen contact pairs, not full-limb immunity.

Contact settings: `solref="0.004 1"`, `solimp="0.95 0.99 0.001"`, margin 6 mm on
skin-derived shapes and explicit pairs. Floor friction and location are unchanged;
contact softness changed, so contact dynamics deliberately differ from before.

Explicit inertias preserve the original body's mass, center of mass and inertia.
All 33 motors, their order, torque limits and gear ratios, joint definitions/ranges,
gravity, integrator and 2 ms timestep are unchanged. The actuator socket/API/CLI,
WebSocket schema and observation UI are unchanged. No balancing controller or
pose correction is added to the physical stepping loop.

## Verification

```sh
.venv/bin/python -m unittest discover -s tests -v
```

- All 24 Python tests pass, including the existing actuator runtime integration.
- Browser verification: accepted shoulder motion renders correctly; a blocked
  through-torso edit shows a contact error and returns the focused numeric field
  to its actual value. See `reports/collision/lab-contact-blocked.jpg`.
- Both shoulders and both hips reject swept paths through other parts, even when
  the endpoint itself is clear. Rejection leaves state and time untouched.
- Foot/floor passage is blocked. Movement away from a contact remains possible.
- At 5 N·m shoulder input in a gravity-free diagnostic, enabling contacts prevents
  penetration of the tested upper-arm/torso pair. Disabling contacts in a separate
  test model gives 94.35 mm penetration (`controlled_motor_contact.json`).
- Six dynamic runs total 30 simulated seconds: gravity alone, quarter-range random
  inputs, three seeds of full-range random inputs, and every motor at its positive
  limit. Contact forces are recorded for all twelve limb segments, with no MuJoCo
  warnings. Peak self-overlap in these runs: 3.57 mm; no overlap above 10 mm.
- Independently reconstructing the GLB bone hierarchy for shoulder rotations about
  all three axes gives maximum limb-anchor mismatch below 0.004 mm. This measures
  anchor alignment, not skin-surface nonintersection.
- Existing actuator API/CLI, every actuator's dynamic response, reset/pause,
  independent joint edits and rest-frame tests remain part of the suite.

Evidence is under `reports/collision/`. The frozen original model is
`reports/collision/model_before.xml`. Rebuild collision assets with:

```sh
.venv/bin/python scripts/build_collision_surfaces.py
```

## Limits

These are rigid convex approximations of the rest skin, not deformable flesh.
Convex hulls may stop contact slightly early; skinning at extreme joint rotations
can still distort a surface. Joint seams intentionally overlap locally. Fingers
are part of each hand envelope, not individual physical fingers. Shoulder/hip
limits are still isotropic ball-joint limits, not a full anatomical joint model.

MuJoCo soft contacts and discrete stepping permit small transient penetration.
The measured test bound is not a guarantee for all possible torque histories.
Position uses sampled sweeps (at most 0.005 rad or 2 mm between samples), not an
exact continuous collision solver. A physical nonpenetration test does not by
itself certify every deformed skin vertex in every possible pose.

## Files

Modified: `physics/model/humanoid_alpha.xml`, `physics/control_console.py`,
`physics/bridge_full/mapping_full.py`, `physics/assumptions.json`,
`viewer/console.js`, `viewer/console.html`, `README.md`,
`tests/test_control.py`, `tests/test_glb_frames.py`.

Added: `physics/collision_guard.py`, `scripts/build_collision_surfaces.py`,
`assets/collision/` (27 hull inputs), `tests/test_collisions.py`, this document,
and validation evidence / pre-change backups under `reports/collision/`.

The current handoff is `../HumanoidAlpha_Delivery/HumanoidAlpha.zip` relative
to the project root. It includes the collision surfaces.
Run `scripts/package_release.py` to rebuild it from the working project.
