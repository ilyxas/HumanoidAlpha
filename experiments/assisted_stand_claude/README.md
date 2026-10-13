# Assisted stand — headless investigation (Claude, 2026-10-13)

Experimental code only. The runtime (`physics/assisted_stand.py`, `control_console.py`), the MJCF,
Animation mode, GLB and Blender assets are **not** modified. Full write-up: `reports/ASSISTED_STAND_CLAUDE.md`.

```sh
.venv/bin/python experiments/assisted_stand_claude/run_suite.py          # ~45 s, 120 s holds
.venv/bin/python experiments/assisted_stand_claude/run_suite.py --quick  # 20 s holds
```

Writes `results.json` next to this file.

- `stand_lib.py`
  - `load_model("mesh" | "flat_sole")`: the flat-sole variant is built in memory. The floor touches one box per foot, sized to the measured flat band of the foot hull, instead of the hull itself.
  - `equilibrium_seed`: lifts only the root until the static floor force equals the body weight.
  - `StanceID`: floating-base inverse-dynamics stance controller. It outputs actuator torques only, clipped to ctrlrange: no xfrc, no qpos writes.
- `run_suite.py`: diagnostics (gravity stiffness vs kp, explicit-PD margin, start impulse, foot contacts, COM-Jacobian authority) and these tests:
  - the existing controller on both variants;
  - StanceID: a 120 s hold, 0.1 s pushes, noisy starts.
