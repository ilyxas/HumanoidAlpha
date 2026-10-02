# ASSISTED Stand Investigation (CURRENT model only)

**Scope:** Investigate only — no controller implementation, no MJCF / physics parameter changes.  
**Source of truth:** `physics/model/humanoid_alpha.xml` + live MuJoCo 3.14 load via `physics/agent_api/env.py` / `physics/control_console.py` (not older copies under `reports/collision/model_before.xml`).  
**Date:** 2026-10-02 (Asia/Jerusalem).

---

## Dump (quick reference)

| Item | Value |
|------|-------|
| **nu actuator names (model order)** | see list below |
| **free joint dims** | `root`: **qpos=7** (xyz + quat wxyz), **qvel=6** (lin 3 + ang 3); damping/armature **0** |
| **timestep** | **0.002 s** (500 Hz) |
| **nbody** | **17** (world + 16 articulated) |
| **nq / nv / njnt / nu** | **47 / 39 / 20 / 33** |
| **key foot body names** | **`foot_l`**, **`foot_r`** (collision geoms `foot_l_geom`, `foot_r_geom`) |
| **total mass** | **≈ 89.044 kg** (mg ≈ 873.5 N) |
| **nkey** | **0** (no MJCF keyframes) |

### Actuator name list (`nu=33`, index order = `./humanoid act` channel order)

```
 0 lumbar_x_motor
 1 lumbar_y_motor
 2 lumbar_z_motor
 3 thoracic_x_motor
 4 thoracic_y_motor
 5 thoracic_z_motor
 6 neck_x_motor
 7 neck_y_motor
 8 neck_z_motor
 9 hip_l_x_motor
10 hip_l_y_motor
11 hip_l_z_motor
12 hip_r_x_motor
13 hip_r_y_motor
14 hip_r_z_motor
15 shoulder_l_x_motor
16 shoulder_l_y_motor
17 shoulder_l_z_motor
18 shoulder_r_x_motor
19 shoulder_r_y_motor
20 shoulder_r_z_motor
21 knee_l_motor
22 knee_r_motor
23 elbow_l_motor
24 elbow_r_motor
25 ankle_l_dp_motor
26 ankle_l_ie_motor
27 ankle_r_dp_motor
28 ankle_r_ie_motor
29 wrist_l_flex_motor
30 wrist_l_dev_motor
31 wrist_r_flex_motor
32 wrist_r_dev_motor
```

Matches `physics/model/actuator_order.json` and `physics/model_meta.json`.

---

## 1. Body hierarchy + free root

```
world
└── pelvis          [freejoint name="root"]   pos₀ ≈ (0, -0.01796, 0.92591)
    ├── torso_lower [ball: lumbar]
    │   └── torso_upper [ball: thoracic]
    │       ├── head [ball: neck]
    │       ├── upperarm_l → lowerarm_l → hand_l
    │       └── upperarm_r → lowerarm_r → hand_r
    ├── thigh_l → calf_l → foot_l
    └── thigh_r → calf_r → foot_r
```

- **Free root on `pelvis`:** underactuated floating base (6 DOF). No root actuators.
- Rest orientation of all bodies at `qpos0` is identity world quat `(1,0,0,0)` (Z-up gravity `0 0 -9.81`).
- Pelvis rest height ≈ **0.926 m**; whole-body COM at rest ≈ `(0, -0.0234, 0.935)`.

---

## 2. Joints / DOFs, actuators, masses, damping, limits

### Joint inventory

| Joint | Type | nq/nv | Limits (rad) | Notes |
|-------|------|-------|--------------|-------|
| `root` | free | 7/6 | unlimited | no damping/armature |
| `lumbar` | ball | 4/3 | cone **0…0.698** (~40°) | isotropic cone |
| `thoracic` | ball | 4/3 | cone **0…0.436** (~25°) | |
| `neck` | ball | 4/3 | cone **0…1.047** (~60°) | |
| `shoulder_l/r` | ball | 4/3 | cone **0…2.618** (~150°) | |
| `hip_l/r` | ball | 4/3 | cone **0…2.094** (~120°) | |
| `elbow_l/r` | hinge | 1/1 | **0…2.618** | anatomical axis (not cardinal) |
| `knee_l/r` | hinge | 1/1 | **-0.087…2.443** | slight hyperextension allowed |
| `ankle_*_dp` | hinge | 1/1 | **-0.349…0.873** | dorsi/plantar |
| `ankle_*_ie` | hinge | 1/1 | **-0.349…0.349** | inversion/eversion |
| `wrist_*_flex` | hinge | 1/1 | **±1.222** | |
| `wrist_*_dev` | hinge | 1/1 | **-0.436…0.611** | |

Ball `qpos` = unit quaternion wxyz; ball `qvel` = 3D angular velocity in joint frame.  
Hinge axes are **mesh-derived unit vectors**, not world XYZ — PD on hinge angle is fine; interpreting “sagittal” for ankles/knees must use the joint axis, not assume `[1,0,0]`.

### Default damping / armature (all articulated DOFs)

From `<default><joint damping="0.08" armature="0.002" …/>` and per-joint repeats:

- **damping = 0.08** N·m·s/rad  
- **armature = 0.002** kg·m²  
- **frictionloss = 0**  
- Free-root 6 DOFs: damping=0, armature=0  

Joint limit solver: `solreflimit="0.002 1"`, `solimplimit="0.95 0.99 0.001"`, `margin="0.02"`.

### Actuators (motors → joint torque)

All 33 are `motor` with `ctrllimited="true"`, `gear` selecting ball axis or 1 for hinges. **ctrl units = N·m** (gear magnitude 1).

| Class | Channels | ctrlrange (N·m) |
|-------|----------|-----------------|
| spine lumbar | 3 | ±35 |
| spine thoracic | 3 | ±30 |
| neck | 3 | ±20 |
| hip L/R | 6 | ±120 |
| shoulder L/R | 6 | ±60 |
| knee L/R | 2 | ±100 |
| elbow L/R | 2 | ±40 |
| ankle dp/ie ×2 | 4 | ±40 |
| wrist flex/dev ×2 | 4 | ±10 |

`ActuatorAPI.validate` **rejects** out-of-range vectors (does not clamp). Console `torque` also rejects OOR. `HumanoidEnv.apply_torques` **clips**.

### Masses / inertias (compiled)

Explicit `<inertial>` on every body (collision rebuild preserved baseline mass properties). Highlights:

| Body | mass (kg) |
|------|-----------|
| pelvis | 11.54 |
| torso_lower | 14.07 |
| torso_upper | 24.12 |
| head | 4.28 |
| thigh_l/r | ≈ 8.34 each |
| calf_l/r | ≈ 3.70 each |
| foot_l/r | ≈ 1.11 each |
| upperarm_l/r | ≈ 2.36 each |
| lowerarm_l/r | ≈ 1.30 each |
| hand_l/r | ≈ 0.70 each |
| **Σ** | **≈ 89.04** |

Diag inertias are stored on each body (see live dump / MJCF); upper torso dominates.

---

## 3. Foot geometry, floor contacts, collision config

### Floor

- `geom name="floor"` plane, size `5 5 0.1`, `friction="0.8 0.1 0.1"`, `condim="3"`, at z=0.
- Global option: `cone="elliptic"`, `impratio="1"`, geom defaults `solref="0.004 1"`, `solimp="0.95 0.99 0.001"`.

### Feet

- Bodies: **`foot_l`**, **`foot_r`**.
- Colliding skin hulls: mesh geoms `foot_l_geom` / `foot_r_geom` (`contype=1`, `conaffinity=1`, `margin=0.006`).
- Distal joint guards `foot_*_joint_guard`: `contype=0 conaffinity=0` (only via explicit `<pair>` with calf).
- At `qpos0`, mesh soles reach **z ≈ −1.59 mm (L)** / **−1.61 mm (R)** — documented in `docs/COLLISIONS.md` and `PositionCollisionGuard.TOLERANCE=0.002`.
- Rest foot body origins ≈ `(±0.123, 0.0015, 0.0745)`.

### Collision policy

- Broad phase: all skin geoms collide with floor and non-adjacent bodies (`contype/conaffinity=1`).
- Adjacent joint seams: **14 explicit `<pair>`** distal-guard ↔ parent geom (shoulder/hip/elbow/wrist/knee/ankle seams).
- **2 `<exclude>`:** `torso_lower`–`thigh_l`, `torso_lower`–`thigh_r` (abdomen/proximal thigh; replaced by thigh distal ↔ torso_lower pairs).
- Head remains a sphere; no finger/toe DOFs.
- Calf sole clearance at rest ≈ **7.6 cm** (not floor-scraping).

### Contacts at `qpos0` (after `mj_forward`)

`ncon=7`, all **floor ↔ foot_*_geom** (mix of slight penetration and near-contact). No self-contacts at rest (covered by tests).

---

## 4. Available runtime state (qpos/qvel, orientation, COM, contacts)

### Layout

**qpos (47):**

| slice | content |
|-------|---------|
| `[0:3]` | root xyz |
| `[3:7]` | root quat wxyz |
| `[7:11]` … ball joints | lumbar, thoracic, neck, shoulder_l, … |
| hinges interleaved | elbows, wrists, knees, ankles (see dump section) |

**qvel (39):** root lin`[0:3]` + ang`[3:6]`, then 3 per ball, 1 per hinge (nv addresses in dump).

### Orientation

- Free-root / body world quat: `data.qpos[3:7]`, `data.xquat[body_id]` (wxyz).
- Console snapshot also streams `body_positions=data.xpos`, `joint_anchors`, `joint_axes`.
- Mapper / viewer use Blender→Three correction in `console-viewer.js` only; physics SoT is MuJoCo Z-up.

### COM

- Whole-body COM: `data.subtree_com[pelvis_id]` (or `[0]` for world subtree) — exposed in `HumanoidDiag.com()`.
- Rest COM XY lies **inside** the bilateral foot support bbox (static support exists); still falls without ankle/hip torque (see § issues).

### Contacts at runtime

- `data.ncon`, `data.contact[i]` (geom pair, dist, pos, frame).
- Forces: `mujoco.mj_contactForce(m, d, i, force6)`.
- `HumanoidEnv.observe()["contact"]`: `ncon`, `floor_contact`, capped `forces_summary`.
- Console WS `state.ncon` only (no force vectors in pose envelope).

### Other useful signals (already in process)

- `data.ctrl` (33), `data.time`, `data.qacc_warmstart`
- Energies via `HumanoidDiag.energies()` if `mj_energy*` enabled (not required for stand PD)

---

## 5. Timestep + simulation loop

**Model:** `timestep="0.002"`, `integrator="implicitfast"`, gravity `(0,0,-9.81)`.

**Owner loop:** `physics/control_console.py::serve`

1. Drain bounded asyncio queue (≤128): either `ActuatorAPI.apply` (API owner) or `Controller.handle` (WS owner).
2. If `mode=='dynamic' and not paused`: accumulate wall time (cap **50 ms** backlog), call `Controller.tick()` once per model dt → `mj_step` + `mj_forward`; nonfinite → `reset` + pause.
3. Broadcast pose/state at **30 Hz**.
4. Sleep ~2 ms.

**API mode** (`--actuator-api` / `./start-actuator-runtime`): starts **dynamic + resume**; zero ctrl still allows gravity fall. CLI disconnect does **not** zero/pause.

**Kinematic mode:** position edits via `joint` + `PositionCollisionGuard`; no `mj_step`.

---

## 6. Existing control paths

| Path | Entry | Effect on `ctrl` | Notes |
|------|-------|------------------|-------|
| **RAW actuator API** | `./humanoid act <33 floats>` → Unix socket JSON `{"values":[…]}` → `ActuatorAPI.apply` | **direct** `data.ctrl[:] = values` | Strict 33 finite; validates limits; persists; opaque `OK` |
| **RAW zero** | `./humanoid zero` | all-zero vector (same apply) | Does **not** pause |
| **Console WS torque** | `{op:'torque', id, value}` | single channel | Requires dynamic; per-actuator |
| **Console zero** | `{op:'zero'}` | zeros + **pause** | Different from API zero |
| **Console mode/reset/pause/resume** | WS ops | mode switch zeros ctrl/qvel | |
| **HumanoidEnv** | `apply_torques` / `step` | clips + steps | Agent/tests path; not the live console loop |

Docs explicitly: **no balance / PD / compensation** in RAW `act` contract (`docs/ACTUATOR_API.md`, `docs/CONTROL_ARCHITECTURE.md`).

---

## 7. Injecting an optional torque controller ABOVE `act` without changing RAW contract

**RAW contract to preserve:**

- CLI syntax and socket payload unchanged.
- Default behavior: accepted vector **is** `data.ctrl` until replaced.
- No silent clamping/compensation when assisted mode is off.
- `humanoid` stays model-agnostic (stdlib only).

### Recommended options (do not implement yet)

**A. External “above act” (zero runtime change — strongest RAW purity)**  
A separate process reads WS `pose`/`state` (or diag), computes τ, calls `./humanoid act …`. Runtime remains RAW. Assisted stand is entirely out-of-process.

**B. Opt-in in-process layer (still RAW by default)**  
Inside `control_console` / `ActuatorAPI` only when e.g. `--assisted-stand` (or env flag) is set:

1. `apply()` stores **feedforward command** `u_cmd` (33,) — same validation as today.  
2. Each `tick()` **before** `mj_step`:  
   - RAW / flag off: `data.ctrl = u_cmd` (bit-identical to current).  
   - Assisted on: `data.ctrl = clip(u_cmd + u_pd_com(q, qd, COM, contacts))` using existing `ctrlrange`.  
3. Do **not** change `humanoid` CLI, socket schema, or default `./start-actuator-runtime` path.  
4. Optional: WS/CLI mode bit that only exists when flag compiled/started — never alter meaning of bare `act` in default experiment launcher.

**C. Do not** overwrite `ctrl` after `apply` in default mode; do not reinterpret the 33 channels as position targets.

Preferred for THIS repo: **A for experiments**, **B if stand must be always-on in the physics process** — both keep RAW when disabled.

---

## 8. Default standing qpos

### What exists

- **No MJCF `<keyframe>`** (`nkey=0`).
- **No separate standing pose** in assets/tests beyond **`model.qpos0` / `inventory.reference_qpos`**.
- Tests reset to `qpos0` and treat it as the exported **rest / feet-on-floor** reference (`test_collisions` asserts rest has no self-penetration; floor tolerance 2 mm for the known −1.61 mm sole).

### `qpos0` summary

- Root: `pos=(0, -0.01795729, 0.92590946)`, `quat=(1,0,0,0)`.
- All ball joints: identity quat; all hinges: **0**.
- This **is** the sensible default standing seed for THIS model: upright T-ish rest with both feet contacting the plane.

### If a “clean soles at z=0” seed is needed (compute, don’t bake into MJCF here)

1. Start from `qpos0`.  
2. `mj_forward`; measure `z_min` of `foot_l_geom`/`foot_r_geom` mesh verts in world.  
3. Add `Δz = -z_min` to `qpos[2]` (root height) so lowest sole point sits at 0 (or leave −1.6 mm and rely on contact as today).  
4. Optionally shift root XY so COM projects to midpoint of foot contact centroids (rest COM already inside support bbox).  
5. Zero `qvel`, `mj_forward`. Do **not** need nonzero joint angles for a first assisted-hold — rest hinges are already 0.

---

## Recommended minimal architecture sketch (ASSISTED PD / COM balance for THIS model)

**Goal:** hold near `qpos0` under gravity using existing 33 torque motors; optional; RAW `act` untouched when off.

```
[ optional high-level intent ]
        │
        ▼
 u_cmd[33]  ← RAW act / zero / (assist off ⇒ ctrl:=u_cmd)
        │
        ▼
 ┌─ AssistedStand (tick @ 500 Hz, before mj_step) ─────────────┐
 │  sensors: qpos, qvel, subtree_com[pelvis], xquat[pelvis],   │
 │           foot contacts (ncon + floor↔foot_* forces/pos)    │
 │  q_des = qpos0 (or sole-corrected stand seed)               │
 │  1) Joint PD on actuated coords only:                       │
 │       hinges: τ = kp*(q_des-q) - kd*qd                      │
 │       balls:  rotate-vector / quat error → 3-axis τ         │
 │       map to motors via existing gear axes (x/y/z motors) │
 │  2) COM balance add-on (small):                             │
 │       e_xy = COM_xy - support_xy (contact centroid / feet)  │
 │       ankle_dp/ie + hip x/y bias from e_xy, ė_xy            │
 │       (axes: use ankle/hip motor frames, not world guess)   │
 │  3) τ = clip(u_cmd + τ_pd + τ_com, ctrlrange)               │
 │  skip / fade if !floor_contact or |tilt| > fail threshold   │
 └─────────────────────────────────────────────────────────────┘
        │
        ▼
 data.ctrl → mj_step (existing loop)
```

**Minimal channel priority for v0 hold:** ankles (25–28), knees (21–22), hips (9–14), light lumbar (0–2); freeze/low-gain arms/wrists/neck toward `qpos0` so they don’t thrash.

**Gains (order-of-magnitude starting point only — tune offline):**  
kp_ankle ~ 50–150, kd ~ 2–10; kp_knee/hip higher; keep τ well below ±40/±100/±120. Model damping is only 0.08 — PD must supply most of the stiffness.

**Fail-safe:** if nonfinite or fallen (pelvis z low / tilt large), zero assist and optionally pause (reuse existing emergency path).

---

## Suspected model issues for standing (do not “fix” here)

1. **Open-loop unstable:** 1 s of zero `ctrl` from `qpos0` → COM drops to ~0.16 m, large root tilt; expected without balance, but means assist must engage **immediately** at resume.  
2. **Soft foot contact + −1.6 mm rest penetration:** compliant `solref=0.004` → mushy ankles; PD may chatter; prefer torque rate limits / low-pass.  
3. **Uniform light joint damping (0.08) / armature (0.002):** little passive help for stand; free root undamped.  
4. **Isotropic ball limits:** not anatomical; hip/ankle “sagittal” assist must use motor gear axes / hinge axes carefully.  
5. **Support vs contact asymmetry:** rest floor contacts cluster with COM inside bbox but not centered on contact centroid (~3–4 cm offset) → constant ankle bias may be needed even at “rest”.  
6. **Convex foot hulls:** contact patches ≠ real foot; CoP estimate from contacts will be noisy.  
7. **No toe joints / narrow effective rocker:** recovery from large lean limited; keep tilt fail threshold conservative.  
8. **89 kg with ankle ±40 N·m:** gravity moment arm budget is tight for large COM offsets (~40/873 ≈ 4.6 cm equivalent at full ankle torque if treated as pure ankle strategy) — **hip strategy required** for anything beyond small sway.  
9. **API vs console zero semantics differ** (pause or not) — assisted mode docs must not conflate them.  
10. **Mapper/viewer frame conjugation** is unrelated to physics stand but can confuse visual “upright” checks; judge stand from MuJoCo `xquat` / COM, not Three.js alone.

---

## Files consulted (current tree)

- `physics/model/humanoid_alpha.xml`
- `physics/model/actuator_order.json`, `physics/model_meta.json`
- `physics/control_console.py`, `physics/actuator_runtime.py`, `humanoid`
- `physics/agent_api/env.py`, `physics/diagnostic_api/diag.py`
- `physics/collision_guard.py`
- `docs/ACTUATOR_API.md`, `docs/CONTROL_ARCHITECTURE.md`, `docs/COLLISIONS.md`
- `tests/test_collisions.py`, `tests/test_actuator_api.py` (behavior expectations)
- Live MuJoCo load + mesh sole / gravity tip measurements (this investigation)

**Explicitly not done:** controller code, MJCF edits, gain tuning runs beyond a 1 s zero-ctrl tip check.
