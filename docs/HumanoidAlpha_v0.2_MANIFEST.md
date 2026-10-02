# HumanoidAlpha_v0.2_MANIFEST

Portable Browser Runtime POC snapshot for transfer (e.g. MacBook Air M4).

## Identity

| Field | Value |
|-------|-------|
| Name | Humanoid Alpha v0.2 Browser Runtime POC (Portable) |
| Packaging date | 2026-09-29 |
| Source (read-only) | `/workspace/blender/body_lab/humanoid_alpha_v0.2/` |
| ZIP extract root | `HumanoidAlpha_v0.2_Portable/` |

## Contents (high level)

- `assets/Xandra.glb` — skinned mesh + armature (~81MB) **REQUIRED**
- `viewer/` — `index.html`, `main.js`, `vendor/three` (three.module.js + addons)
- `physics/` — MJCF (`model/humanoid_alpha.xml` + pre_anatomical), `agent_api`, `diagnostic_api`, `bridge_full` (+ `calibration_rest.json`), `ws_bridge.py`, metadata JSON
- `scripts/` — portable `run_demo.sh`, capture `*.mjs`, audit/inspect helpers
- `shots/` — representative PNG + stats JSON
- `reports/` — POC_REPORT, VIEWER_MANUAL, EXPORT_REPORT, etc.
- `package.json` / `package-lock.json` — three **0.170.0**, puppeteer-core (optional)
- `requirements.txt` — pinned Python deps
- `README.md`, `README_MAC.md`, `CHANGES_FROM_LIVE.md`, `RESTORE_TEST_REPORT.md`, this MANIFEST

## Excluded

`node_modules/`, `__pycache__/`, `.venv`, `Xandra_F4_BodyLab_export_src.blend`, any other `.blend`

## Versions / external deps

| Dep | Version / note |
|-----|----------------|
| Python | Prefer 3.11–3.13 (packaged against 3.13.5 on Linux) |
| mujoco | ==3.14.0 (pip wheel; Apple Silicon via pip — verify locally) |
| numpy | ==2.5.3 |
| websockets | ==17.1 |
| absl-py / etils / glfw / PyOpenGL | pinned in requirements.txt |
| three.js | 0.170.0 vendored under `viewer/vendor/three` |
| Node / puppeteer-core | Optional (capture scripts only) |
| Browser | Safari/Chrome; WebGL |
| Blender | **NOT required at runtime** |

## Ports

- HTTP: `127.0.0.1:8787` (`python -m http.server` from project root)
- WebSocket: `ws://127.0.0.1:8765` (`physics/ws_bridge.py`)

## Integrity

See sibling `HumanoidAlpha_v0.2_SHA256.txt` for zip and key asset hashes.
