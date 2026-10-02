# ASSISTED Stand Report

**Date:** 2026-10-02 (Asia/Jerusalem)  
**Model:** CURRENT `physics/model/humanoid_alpha.xml` (no mass/limit/collision edits)  
**Validation:** `reports/assisted_stand/metrics.json` → **PASS** (best continuous upright **10.05 s**; was ~5.90 s)

---

## 1. Architecture

```
u_cmd[33]  ← ./humanoid act | zero   (validated, same RAW contract)
        │
        ▼
 ┌─ AssistedStandController (only if assisted=True) ─────────────┐
 │  observe: qpos, qvel, subtree_com[pelvis], xquat[pelvis],      │
 │           floor↔foot_* contacts                                 │
 │  1) Posture PD → q_des = stand seed (qpos0 at enable)         │
 │       hinges: τ = kp*(q_des−q) − kd*qd                        │
 │       balls:  quat error → 3-axis τ on gear x/y/z motors      │
 │  2) COM balance: J_subtreeCom^T · f_xy (+ modest f_z)          │
 │       e_xy = (support_xy + offset0) − COM_xy                   │
 │       e_z  = z_des − pelvis_z ; vz = root qvel[2]              │
 │       support = midpoint(foot_l, foot_r xpos)                   │
 │       applied only on ankle/knee/hip/lumbar channels            │
 │  3) τ = clip(u_cmd + fade·(τ_pd+τ_com), ctrlrange)            │
 │  fail-safe: fade→0 if tilt large / pelvis z low / no foot con │
 └───────────────────────────────────────────────────────────────┘
        │
        ▼
 data.ctrl → mj_step → mj_forward   (existing control_console loop)

When assisted=False: ActuatorAPI.apply writes data.ctrl := values;
tick only mj_step (+ mj_forward). Bit-identical RAW.
```

`reset_to_stand()` (qpos←qpos0, qvel←0, `mj_resetData`+`mj_forward`) runs **only** on assist enable / `humanoid-assist reset` — never every tick.

---

## 2. Files

| Path | Role |
|------|------|
| `physics/assisted_stand.py` | `AssistedStandController` |
| `physics/control_console.py` | `--assisted-stand`, `assisted`/`assist`/`u_cmd`, tick integrate |
| `physics/actuator_runtime.py` | `u_cmd` when assisted; `{"assist":...}` side channel |
| `scripts/assist_cli.py` + `humanoid-assist` | `on`/`off`/`reset`/`status` |
| `scripts/launch.py` / `START.command` | mode `assisted` |
| `scripts/validate_assisted_stand.py` | headless PASS/FAIL |
| `docs/ASSISTED_STAND.md` | short how-to |
| `reports/assisted_stand/*` | metrics, CSV, plot, PNGs |
| `humanoid` | **unchanged** (act/zero only) |

---

## 3. RAW start method

```bash
./START.command experiment
./humanoid act <33 finite floats>   # → data.ctrl directly
./humanoid zero
```

Or `./start-actuator-runtime`. No assist keys; socket accepts only `{"values":[…]}`.

---

## 4. ASSISTED start method

```bash
./START.command assisted
# ≡ control_console --actuator-api --assisted-stand
# On start: assist ON, reset_to_stand once, dynamic+resume, u_cmd=0

./humanoid-assist status|on|off|reset
./humanoid act … / ./humanoid zero   # still valid; become feedforward u_cmd
```

Socket (when capability present):

- `{"values":[…]}` — RAW vector (→ `u_cmd` if assisted, else `ctrl`)
- `{"assist":"on"|"off"|"reset"|"status"}` — assist control

---

## 5. State (observed)

- `qpos` / `qvel` (full)
- `subtree_com[pelvis]` (whole-body COM)
- `xquat[pelvis]` → tilt from body-Z vs world-Z
- Foot floor contacts: count of floor↔`foot_l_geom`/`foot_r_geom`
- Support XY: midpoint of `foot_l`/`foot_r` body xpos
- Rest COM offset vs support recorded at `reset_to_stand`

Diagnostics dict: `com_xy`, `support_xy`, `tilt`, `ncon_feet`, `tau_norm`, `fade`, `enabled`, `pelvis_z`.

---

## 6. Torques

- Units: N·m (motor `ctrl` = torque; gear magnitude 1)
- Composition: `τ = clip(u_cmd + fade·(τ_posture_PD + τ_COM_jac), actuator_ctrlrange)`
- Priority: higher posture kp on ankles/knees/hips/lumbar; low on arms/wrists/neck
- COM virtual force only in XY; mapped via `mj_jacSubtreeCom` onto ankle/knee/hip/lumbar DOFs
- Tuned (CEM + modest vertical COM, `mj_forward` after each `mj_step` to match console):

| Class | kp (approx) | kd |
|-------|-------------|-----|
| ankle | 139.3 | 5.64 |
| knee | 294.5 | 20.4 |
| hip | 128.5 | 8.0 |
| lumbar | 38.1 | 3.0 |
| thoracic / upper | ~36 | low |
| arms/neck/wrists | low | low |
| KP_COM / KD_COM | 4475 / 134 | (virtual force N/m, N·s/m) |
| KP_Z / KD_Z | 204 / 12.8 | (vertical COM via same Jac) |

---

## 7. Validation results

Script: `scripts/validate_assisted_stand.py`  
Artifacts: `reports/assisted_stand/`

| Metric | Value |
|--------|-------|
| **Result** | **PASS** |
| Best continuous upright | **10.054 s** (was 5.896 s) |
| Time to fall (upright lost) | **~10.06 s** |
| Meets ≥10 s retune target | **yes** (model still caps ~10 s; not indefinite) |
| Criteria | pelvis z ≥ 0.70 m and tilt ≤ 0.55 rad |
| ASSISTED horizon | 15 s sim |
| RAW zero from stand (2 s) | pelvis z ≈ 0.11 m, tilt ≈ 1.86 rad (fallen) |
| Images | `stand_t0.png`, `stand_t5.png`, `stand_t10.png` |
| Plot / CSV | `plot.png`, `timeseries.csv` |

During the hold window, pelvis z stays ~0.88–0.93 m with modest lean before eventual sink/tip — clearly upright vs RAW collapse within ~1 s. Gains alone could not beat ~6 s; adding modest vertical COM (`KP_Z`/`KD_Z` on the existing Jac) plus a light CEM retune reached ~10 s. Further CEM search did not yield stable ≥15 s — ankle ±40 N·m and soft contacts remain the binding limits.

Unit smoke: `tests.test_actuator_api.VectorTests` OK; RAW `apply` still sets `ctrl` when `assisted=False`; tick does not rewrite `ctrl` in RAW.

---

## 8. Limitations

1. **Not indefinite stand.** After ~10 s the robot sinks/tips and falls; fail-safe fades assist. ≥15 s was not achieved with honest PD+Jac torques on this model.
2. **Gain-sensitive.** Tuned for THIS model + console (`mj_step`+`mj_forward`) loop; other timesteps/integrators need retuning.
3. **Soft contacts / convex feet.** CoP from contacts is noisy; support uses foot body midpoint instead.
4. **Ankle torque budget (±40 N·m vs ~89 kg).** Large COM offsets cannot be recovered by ankle strategy alone; hip Jac helps but recovery envelope is small (~few cm).
5. **No stepping / arm windmill.** Pure hold near `qpos0`.
6. **Feedforward `u_cmd` can fight balance** if the operator injects large torques while assist is on.
7. **Sole lift off by default.** CEM gains assume raw `qpos0` (known ~−1.6 mm sole penetration). Optional `sole_lift=True` exists but degrades this tune.

---

## 9. Model issues (documented only — not changed)

From investigation + this work (see also `reports/ASSISTED_STAND_INVESTIGATION.md`):

1. Open-loop unstable at `qpos0` (RAW zero falls in &lt;1–2 s).
2. Soft foot `solref` + rest sole penetration → mushy contact; PD chatter risk.
3. Uniform joint damping 0.08 / armature 0.002 — little passive stand help; free root undamped.
4. Ankle ±40 N·m vs mg≈873 N → ~4–5 cm pure-ankle lever budget.
5. Convex foot hulls; contact patch ≠ anatomical foot.
6. No toe DOFs / narrow rocker → limited recovery from large lean.
7. Rest COM vs foot-midpoint offset (~2.5 cm) — controller stores offset0 at reset.
8. Isotropic ball joint limits — not anatomical; Jac mapping preferred over hand-signed world axes.

---

## 10. Debug console UI

`./START.command debug` always constructs `AssistedStandController`. The console has a third tab **Assisted** next to Position / Torque:

- Click **Assisted** → `mode=dynamic` + assist ON + `reset_to_stand` once + **auto-resume**.
- Sliders write **`u_cmd`** (additive intent); tick applies `τ = clip(u_cmd + fade·(τ_pd+τ_com))`.
- **Torque** disables assist (RAW `d.ctrl`); **Position** disables assist and freezes time.
- Snapshot exposes `assisted` + `u_cmd` for UI highlighting / slider readout.
- Inventory includes `assisted_capability: true`.

## Honesty note

PASS means ≥5 s continuous meaningful upright under the stated criteria with honest PD+Jac torques and no physics cheats. It does **not** claim infinite balance or production-ready locomotion.
