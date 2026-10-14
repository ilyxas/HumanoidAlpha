# ASSISTED stand — how to

Optional balance layer **above** RAW `./humanoid act` and above debug Torque mode.
When assist is off, behaviour is bit-identical RAW (`data.ctrl` ← command).

Capability is **always available** in the developer console (`./START.command debug`).
`./START.command assisted` still auto-enables assist for the experiment+CLI path, but it is no longer a launcher menu item. Menu item 3 is HARNESS (`./START.command harness`). Debug → Assisted is unchanged.

## Debug console (recommended)

```bash
./START.command debug
# Open the Body Control page → click **Assisted** (next to Position / Torque)
# Assist seeds stand once and auto-resumes simulation.
# Actuator sliders write additive u_cmd (PD+COM still applied each tick).
# Position / Torque tabs disable assist (Torque = RAW).
```

## Start RAW experiment (unchanged)

```bash
./START.command experiment
# or: ./start-actuator-runtime
./humanoid act <33 floats>
./humanoid zero
```

## Start ASSISTED experiment + CLI

```bash
./START.command assisted
# physics starts with --actuator-api --assisted-stand:
#   dynamic + resume, assist ON, reset_to_stand once, u_cmd=0

./humanoid-assist status
./humanoid-assist reset   # re-seed stand pose (qpos write once)
./humanoid-assist off     # back to RAW (ctrl := last u_cmd)
./humanoid-assist on      # enable + reset_to_stand again

# Feedforward still via RAW CLI (added to PD/COM, then clipped):
./humanoid act <33 floats>
./humanoid zero
```

## Validate headless

```bash
.venv/bin/python scripts/validate_assisted_stand.py
# → reports/assisted_stand/metrics.json (PASS/FAIL), plot.png, stand_t*.png
# Success: best continuous upright ≥ 5 s; retune target ≥ 10 s when model allows.
```

## Architecture (one line)

`u_cmd` (act/zero/console sliders) + AssistedStand PD/COM(+modest vertical) → clip to ctrlrange → `data.ctrl` → `mj_step`.  
No MJCF edits. No `qpos` writes during sim (only at enable/reset).

## Stand v2 (experimental, branch `exp/assisted-stand-claude`)

```bash
./START.command stand-v2   # observation + actuator CLI, stand v2 on (menu 5)
./START.command debug-v2   # developer console; the Assisted button uses stand v2 (menu 6)
```

The stand-v2 and debug-v2 modes differ from the existing Assisted in two ways:
- **Model:** a flat-sole contact variant, built in memory by `physics/stand_v2.py:load_flat_sole_model`. The floor touches one box per foot instead of the convex foot hull. `physics/model/humanoid_alpha.xml` is unchanged.
- **Controller:** `StandV2Assist`, a floating-base inverse-dynamics stance with the centre of pressure kept inside the soles. Its start pose is the equilibrium seed rather than raw qpos0.

Like the existing Assisted, it outputs actuator torques only, clipped to ctrlrange: no xfrc and no qpos writes except the seed. `./humanoid act` still adds `u_cmd`, and `humanoid-assist on|off|reset|status` work the same way. Past 45° of torso tilt or below 0.55 m of pelvis height the assist fades to zero, and `humanoid-assist reset` re-seeds the pose.

Status: holds 120 s undisturbed and survives 0.1 s pushes of 5 N·s. It falls after 10 N·s backward or sideways. Why it was needed: `reports/ASSISTED_STAND_CLAUDE.md`.
