# Mass model: 89 kg baseline vs per-body 60 kg

- Branch: `exp/mass-60kg-per-body`
- main (untouched parent): `30fc2e69dfc527397764fb02b1ff061ca46604c4` at generation time. The experiment commit is the commit that adds this file on `exp/mass-60kg-per-body`; it is not main.
- Baseline loaded from: `git show main:physics/model/humanoid_alpha.xml`
- Experimental model: `physics/model/humanoid_alpha.xml`

## Inertia method

Every body already had an explicit inertial element whose compiled mass was finite and positive and whose diaginertia (principal moments about the existing COM) was finite, positive, and satisfied the triangle inequalities, so the tensor is positive definite and physically realizable. No body needed a fallback. For each body independently, s = m_intended / m_compiled_baseline, new mass = m_intended (from physics/assumptions.json masses_kg), new diaginertia = s * old diaginertia. inertial pos and quat were not edited. Scaling a valid principal inertia by a positive scalar preserves the COM, the principal-axis directions, the radii of gyration, positive-definiteness, and the triangle inequalities. inertiafromgeom remains auto; the explicit inertial overrides geom density (the head sphere still carries its old density attribute, which does not add mass while the inertial element is present — compiled head mass matches masses_kg). Collision geoms were not edited.

## Before table (compiled baseline vs assumptions.json masses_kg)

| body | current mass kg | intended mass kg | ratio current/intended | baseline inertia valid |
|---|---:|---:|---:|:---:|
| pelvis | 11.53826890 | 9.28104575 | 1.243208 | True |
| torso_lower | 14.07392644 | 6.86274510 | 2.050772 | True |
| torso_upper | 24.12119451 | 10.98039216 | 2.196752 | True |
| head | 4.27877381 | 5.29411765 | 0.808213 | True |
| upperarm_l | 2.36011309 | 1.83006536 | 1.289633 | True |
| lowerarm_l | 1.30151421 | 1.04575163 | 1.244573 | True |
| hand_l | 0.70175441 | 0.39215686 | 1.789474 | True |
| upperarm_r | 2.36011580 | 1.83006536 | 1.289635 | True |
| lowerarm_r | 1.30151693 | 1.04575163 | 1.244576 | True |
| hand_r | 0.70175436 | 0.39215686 | 1.789474 | True |
| thigh_l | 8.34167723 | 6.53594771 | 1.276277 | True |
| calf_l | 3.70287501 | 3.03921569 | 1.218365 | True |
| foot_l | 1.10784941 | 0.94771242 | 1.168972 | True |
| thigh_r | 8.34168042 | 6.53594771 | 1.276277 | True |
| calf_r | 3.70287427 | 3.03921569 | 1.218365 | True |
| foot_r | 1.10784895 | 0.94771242 | 1.168972 | True |
| **TOTAL** | **89.04373775** | **60.00000000** | **1.484062** | |

## After table (compiled experimental model)

| body | compiled mass kg | intended kg | abs err | inertia valid |
|---|---:|---:|---:|:---:|
| pelvis | 9.28104575 | 9.28104575 | 0.000e+00 | True |
| torso_lower | 6.86274510 | 6.86274510 | 0.000e+00 | True |
| torso_upper | 10.98039216 | 10.98039216 | 5.329e-15 | True |
| head | 5.29411765 | 5.29411765 | 0.000e+00 | True |
| upperarm_l | 1.83006536 | 1.83006536 | 2.220e-16 | True |
| lowerarm_l | 1.04575163 | 1.04575163 | 0.000e+00 | True |
| hand_l | 0.39215686 | 0.39215686 | 5.551e-17 | True |
| upperarm_r | 1.83006536 | 1.83006536 | 2.220e-16 | True |
| lowerarm_r | 1.04575163 | 1.04575163 | 0.000e+00 | True |
| hand_r | 0.39215686 | 0.39215686 | 5.551e-17 | True |
| thigh_l | 6.53594771 | 6.53594771 | 0.000e+00 | True |
| calf_l | 3.03921569 | 3.03921569 | 4.441e-16 | True |
| foot_l | 0.94771242 | 0.94771242 | 0.000e+00 | True |
| thigh_r | 6.53594771 | 6.53594771 | 0.000e+00 | True |
| calf_r | 3.03921569 | 3.03921569 | 4.441e-16 | True |
| foot_r | 0.94771242 | 0.94771242 | 0.000e+00 | True |
| **TOTAL** | **60.00000000** | **60.00000000** | **7.105e-15** | |

## Invariant checks (compiled baseline vs compiled experimental)

| check | result | detail |
|---|---|---|
| nq | PASS | 47 -> 47 |
| nv | PASS | 39 -> 39 |
| nu | PASS | 33 -> 33 |
| actuator_names_order | PASS | n=33 first=lumbar_x_motor last=wrist_r_dev_motor |
| ctrlrange | PASS | max abs diff 0.0 |
| jnt_range | PASS | max abs diff 0.0 |
| jnt_axis | PASS | max abs diff 0.0 |
| qpos0 | PASS | max abs diff 0.0 |
| gravity | PASS | [0.0, 0.0, -9.81] |
| timestep | PASS | 0.002 |
| body_ipos | PASS | max abs diff 0.0 |
| body_iquat | PASS | max abs diff 0.0 |
| radii_of_gyration_I_over_m | PASS | max abs diff 1.0408340855860843e-17 |
| collision_geom_fingerprint | PASS | ngeom 29 |
| dof_damping | PASS | max abs diff 0.0 |
| dof_armature | PASS | max abs diff 0.0 |
| per_body_mass_matches_masses_kg | PASS | atol=1e-08 max abs err=5.329e-15 |
| total_mass_60 | PASS | compiled total 60.000000000000 |

## Behavioral A/B

Identical `qpos0`, zero initial velocity. No gain changes. A = main-branch compiled model (~89.0437 kg). B = experimental model.

### Zero-control fall (2.0 s, ctrl=0, harness off)

| t s | A pelvis z | B pelvis z | A tilt rad | B tilt rad |
|---:|---:|---:|---:|---:|
| 0.0 | 0.925909 | 0.925909 | 0.000000 | 0.000000 |
| 0.5 | 0.536810 | 0.550302 | 0.342565 | 0.291427 |
| 1.0 | 0.129355 | 0.133932 | 1.195920 | 0.902541 |
| 1.5 | 0.126534 | 0.127272 | 1.830329 | 1.531935 |
| 2.0 | 0.111958 | 0.129597 | 1.856248 | 1.394899 |
- A: first pelvis z < 0.55 s at t=0.4820000000000004, min z=0.111688, max tilt=1.884512
- B: first pelvis z < 0.55 s at t=0.5020000000000003, min z=0.120673, max tilt=1.544922

### HARNESS, zero human command (5.0 s, gains unchanged)

Existing gains were tuned against ~89 kg (`KP_Z=30000` N/m, comment in `physics/harness.py` cites ~874 N). Not retuned.

| t s | A z | B z | A tilt | B tilt | A wrench | B wrench | A Fz | B Fz |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.925909 | 0.925909 | 0.000000 | 0.000000 | 0.000 | 0.000 | 0.000 | 0.000 |
| 1.0 | 0.901454 | 0.907446 | 0.050734 | 0.054075 | 741.834 | 654.284 | 736.388 | 648.155 |
| 2.0 | 0.902570 | 0.906764 | 0.055175 | 0.036873 | 717.817 | 577.068 | 710.917 | 573.328 |
| 3.0 | 0.899521 | 0.908114 | 0.054637 | 0.035573 | 789.405 | 536.819 | 783.233 | 532.996 |
| 4.0 | 0.905277 | 0.908256 | 0.052428 | 0.035461 | 623.490 | 533.827 | 616.284 | 529.993 |
| 5.0 | 0.902124 | 0.907886 | 0.053457 | 0.035872 | 730.927 | 544.724 | 724.557 | 540.880 |
- A: weight=873.519 N, predicted static sag mg/KP_Z=0.029117 m, end sag=0.023785 m, min z=0.894169, max tilt=0.116775, validator hold (min z>0.75 and max tilt<0.6)=True
- B: weight=588.600 N, predicted static sag mg/KP_Z=0.019620 m, end sag=0.018024 m, min z=0.905654, max tilt=0.132457, validator hold (min z>0.75 and max tilt<0.6)=True

### Lumbar/thoracic open-loop torque (harness off)

`lumbar_x_motor=20` N·m and `thoracic_x_motor=15` N·m for 0.8 s. Other actuators 0. Angle is the ball-joint rotation magnitude.

| t s | A lumbar | B lumbar | A thoracic | B thoracic | A z | B z |
|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.925909 | 0.925909 |
| 0.2 | 0.678791 | 0.679026 | 0.416634 | 0.417056 | 0.872011 | 0.869482 |
| 0.4 | 0.679451 | 0.679397 | 0.417693 | 0.417041 | 0.569272 | 0.577612 |
| 0.6 | 0.679140 | 0.678710 | 0.419212 | 0.416920 | 0.451742 | 0.487747 |
| 0.8 | 0.678501 | 0.679027 | 0.416663 | 0.418409 | 0.232772 | 0.272447 |

### Existing limb torque test: knee_l 70 N·m with HARNESS (0.8 s)

Same open-loop knee torque the harness validator uses (`KNEE_TORQUE=70`). Gains unchanged.

| t s | A knee rad | B knee rad | A sole_l z | B sole_l z | A left floor contacts | B left floor contacts | A z | B z |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.000000 | 0.000000 | -0.001589 | -0.001589 | 4 | 4 | 0.925909 | 0.925909 |
| 0.2 | 2.082967 | 2.082983 | 0.318009 | 0.332223 | 0 | 0 | 0.901235 | 0.910032 |
| 0.4 | 2.082862 | 2.082987 | 0.325574 | 0.332415 | 0 | 0 | 0.898450 | 0.906885 |
| 0.6 | 2.082853 | 2.082993 | 0.421167 | 0.421720 | 0 | 0 | 0.895082 | 0.906314 |
| 0.8 | 2.082904 | 2.083033 | 0.508425 | 0.495735 | 0 | 0 | 0.898247 | 0.906663 |
- A lifted (left foot clear, sole>0.15 m, right foot still in floor contact) at t=0.8: True
- B lifted at t=0.8: True

The 20/15 N·m spine command and the 70 N·m knee command both reach essentially the same joint angle by 0.2 s on A and B (spine angles sit on the ball-cone stops: lumbar limit 0.698 rad, thoracic limit 0.436 rad). That comparison is contact/limit dominated, so it does not show the inertia change. A smaller open-loop probe is below.

### Sub-limit probe (not a retune): lumbar_x=2 N·m, thoracic_x=2 N·m, harness off; knee_l=8 N·m with harness

| t s | A lumbar | B lumbar | A thoracic | B thoracic | A z | B z |
|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.925909 | 0.925909 |
| 0.2 | 0.677738 | 0.678489 | 0.417846 | 0.396879 | 0.881088 | 0.889117 |
| 0.4 | 0.679931 | 0.687140 | 0.059892 | 0.101695 | 0.581003 | 0.586619 |

| t s | A knee rad | B knee rad | A sole_l z | B sole_l z | A pelvis z | B pelvis z |
|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.000000 | 0.000000 | -0.001589 | -0.001589 | 0.925909 | 0.925909 |
| 0.2 | 1.091164 | 1.199505 | 0.024785 | 0.043840 | 0.893957 | 0.904428 |
| 0.3 | 1.395323 | 1.610323 | 0.061379 | 0.138739 | 0.896760 | 0.907574 |

## Provenance

PROVEN, in this repo: compiled `main` total mass is 89.043737750521 kg. `reports/collision/model_before.xml` compiled total is 89.043737750485 kg. Max per-body absolute mass difference is 3.743e-11 kg. So the current explicit inertials are the pre-collision compiled masses, not a new mesh-density integration.

PROVEN, in this repo: `scripts/build_collision_surfaces.py` loads `reports/collision/model_before.xml`, writes an `<inertial>` per body from `model.body_mass`, `body_inertia`, `body_ipos`, `body_iquat`, and asserts the rebuilt model matches those arrays (`rtol=1e-10`). `docs/COLLISIONS.md` states that explicit inertias preserve the original mass, COM and inertia. The pre-change model has `inertiafromgeom="true"` and capsule/sphere/box geoms whose densities are the values in `physics/assumptions.json`.

PROVEN, compiled `model_before.xml` mass equals `density * MuJoCo geom volume` (not the intended mass):

| body | intended | density * geom volume | compiled | volume kind |
|---|---:|---:|---:|---|
| pelvis | 9.281046 | 11.538269 | 11.538269 | capsule (cylinder length = fromto, caps extra) |
| torso_lower | 6.862745 | 14.073926 | 14.073926 | capsule (cylinder length = fromto, caps extra) |
| torso_upper | 10.980392 | 24.121195 | 24.121195 | capsule (cylinder length = fromto, caps extra) |
| head | 5.294118 | 4.278774 | 4.278774 | sphere |
| upperarm_l | 1.830065 | 2.360113 | 2.360113 | capsule (cylinder length = fromto, caps extra) |
| lowerarm_l | 1.045752 | 1.301514 | 1.301514 | capsule (cylinder length = fromto, caps extra) |
| hand_l | 0.392157 | 0.701754 | 0.701754 | capsule (cylinder length = fromto, caps extra) |
| upperarm_r | 1.830065 | 2.360116 | 2.360116 | capsule (cylinder length = fromto, caps extra) |
| lowerarm_r | 1.045752 | 1.301517 | 1.301517 | capsule (cylinder length = fromto, caps extra) |
| hand_r | 0.392157 | 0.701754 | 0.701754 | capsule (cylinder length = fromto, caps extra) |
| thigh_l | 6.535948 | 8.341677 | 8.341677 | capsule (cylinder length = fromto, caps extra) |
| calf_l | 3.039216 | 3.702875 | 3.702875 | capsule (cylinder length = fromto, caps extra) |
| foot_l | 0.947712 | 1.107849 | 1.107849 | box |
| thigh_r | 6.535948 | 8.341680 | 8.341680 | capsule (cylinder length = fromto, caps extra) |
| calf_r | 3.039216 | 3.702874 | 3.702874 | capsule (cylinder length = fromto, caps extra) |
| foot_r | 0.947712 | 1.107849 | 1.107849 | box |

PROVEN by executing the original generator, which is **not** in this git history (the repo imported the already-built MJCF in commit `75ef16b`): `/workspace/blender/body_lab/humanoid_alpha/scripts/02_build_mjcf.py` sets `density = intended_mass / capsule_volume(length, r)` with `h = max(length - 2r, 1e-6)` and `V = pi r^2 h + 4/3 pi r^3`. That treats `length` as the outer capsule length **including** both hemispherical caps. MuJoCo `fromto` capsules use `length` as the cylinder axis and add the caps outside it, so the geom is heavier whenever `length > 2r`. Pelvis uses a hard-coded length 0.12 m (shorter than the diameter, so the density volume collapses to a sphere) while the geom is a short capsule. `torso_lower` length 0.126 m is shorter than diameter 0.18 m, same sphere-vs-capsule gap (the largest torso error). Head density uses that capsule formula on `neck_head` length 0.2085 m, but the geom is a sphere, so the head is the only body **lighter** than intended. Feet use the capsule formula for density and a box geom. Recomputing every `assumptions.json` density from that function and `measurements.json` segment lengths matched bit-exactly (max relative error 0) on this machine.

HYPOTHESIS: none required for the 60→89 kg gap. The collision-surface rebuild did not create the extra mass; it preserved it. A non-uniform global scale of the 89 kg model would not recover `masses_kg`, because head must go up (ratio 0.808) while `torso_upper` must go down (ratio 2.197).

## Failures / unexpected

- HARNESS still meets the existing 5 s hold criterion on the 60 kg model (min z=0.9057, max tilt=0.1325) with unchanged gains. End sag 0.0180 m vs baseline 0.0238 m. This is not a retune; the spring was already strong relative to either weight.
- tests/test_collisions.py::test_original_dynamics_properties_and_actuators_preserved asserts compiled body_mass/body_inertia still match reports/collision/model_before.xml. That assertion is expected to FAIL after this mass correction. It was not edited. Measured failure: body_mass, 16/17 bodies differ, max abs diff 13.14080235 kg (torso_upper 24.121195 -> 10.980392).
- Existing tests were run and not retuned. `python -m unittest discover -s tests`: 26 tests, 25 passed, 1 failed (the collision mass-preservation test above). Human-control test passed. `scripts/validate_harness.py` all_pass true on the 60 kg model with unchanged gains: harness min pelvis z 0.905654, end z 0.907886, end tilt 0.035872 rad, max tilt 0.132457 rad, raw fall to z<0.55 at 0.502 s, knee 70 N·m lift still clears the left foot (sole_l 0.4957 m at 0.8 s) with right foot on the floor. Those validators rewrite reports/harness, reports/human_control/validation.json (xfrc_peak_abs 1097.646 -> 884.542 N) and reports/collision/controlled_motor_contact.json (contact_off_peak_m 0.094349 -> 0.102681). Those rewrites were restored and are not part of this branch; the numbers are recorded here.

