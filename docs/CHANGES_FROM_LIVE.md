# CHANGES_FROM_LIVE — Humanoid Alpha v0.2 Portable

Packaging date: 2026-09-29 (Asia/Jerusalem).  
Source live project: `/workspace/blender/body_lab/humanoid_alpha_v0.2/`  
**Live project was not modified.** All edits below are in this portable copy only.

## Exclusions (not copied)

| Item | Reason |
|------|--------|
| `node_modules/` | Regenerable; viewer ships `viewer/vendor/three` for runtime |
| `__pycache__/`, `*.pyc` | Regenerable |
| `.venv` | Create fresh with `requirements.txt` |
| `Xandra_F4_BodyLab_export_src.blend` (~152MB) | Export source; Blender not needed at runtime |

## Files touched (portability only)

| File | Change |
|------|--------|
| `scripts/run_demo.sh` | `PY` default: prefer `$ROOT/.venv/bin/python`, else `python3` (was absolute knee_physics venv path) |
| `scripts/05_capture_posed_fall.mjs` | `PY = process.env.PY \|\| VIRTUAL_ENV/bin/python \|\| 'python3'` |
| `scripts/07_capture_fall_early.mjs` | same as above |
| `scripts/01_audit_and_export.py` | `V02 = Path(__file__).resolve().parents[1]` |
| `scripts/02_inspect_glb.py` | same |
| `physics/model_meta.json` | `model_path` → `model/humanoid_alpha.xml` + `model_path_note` with original absolute path. Runtime `env.py` already uses `Path(__file__)` — **no physics behavior change** |
| `physics/anatomical_ranges.json` | Added `joints_dof_csv_note`; kept historical absolute `joints_dof_csv` as FACT of original export (not opened at runtime) |
| `package.json` | Pinned `three` to exact `0.170.0` (was `^0.170.0`; lock already resolved 0.170.0) |
| `README.md` | Rewritten with portable paths; points to README_MAC.md |

## Historical absolute paths intentionally kept

Reports under `reports/*.json` and `reports/blender_export_stdout.txt` retain original `/workspace/blender/...` paths as **FACT of the original export** (audit trail). They are not used by runtime.

## Not modified (behavior / math)

- `physics/agent_api/env.py`, `physics/ws_bridge.py`, `physics/bridge_full/mapping_full.py`
- `physics/model/*.xml`, `physics/bridge_full/calibration_rest.json`
- `viewer/main.js`, `viewer/index.html`, `viewer/vendor/three/**`
- Pose mapping algorithms, MuJoCo model, calibration numbers

## Runtime path verification (unchanged code)

- `env.py`: `ROOT = Path(__file__).resolve().parents[1]` → `model/humanoid_alpha.xml`
- `ws_bridge.py`: `ROOT = Path(__file__).resolve().parent`; calib + model relative
- `mapping_full.py`: `CALIB_PATH` next to itself
- `viewer/main.js`: `loader.load('../assets/Xandra.glb', ...)`
- `viewer/index.html`: importmap `./vendor/three/...`
