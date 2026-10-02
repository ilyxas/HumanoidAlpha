# RESTORE_TEST_REPORT — Humanoid Alpha v0.2 Portable

**Date:** 2026-09-29 Asia/Jerusalem (box local)  
**Host:** Linux VM (packaging machine), Python 3.13.5  
**Explicitly: NOT tested on macOS.**

## Procedure

1. Fresh tree: `/workspace/restore_tests/HumanoidAlpha_v0.2_clean/` (copy of portable staging; outside Body Lab live project).
2. `python3 -m venv .venv` && `pip install -r requirements.txt` (exit 0).
3. `PYTHONPATH` unset; `sys.path` had no `/workspace/blender/...` entries during import smoke.
4. Smoke commands run from cleanroom with `.venv/bin/python`.

## Results

| Criterion | Result | Evidence |
|-----------|--------|----------|
| `pip install -r requirements.txt` | **PASS** | mujoco 3.14.0, numpy 2.5.3, websockets 17.1 installed |
| Import HumanoidEnv, load model, reset, step, extract_full_pose | **PASS** | nq=47 nu=33 bones=23 apply_order=23; DEFAULT_MODEL under cleanroom |
| Fall short sim advances | **PASS** | 50 zero-torque steps; pelvis xpos moved (z≈0.895) |
| ws_bridge listens + pose broadcast | **PASS** | Listen `127.0.0.1:8765`; client recv 5 msgs; `type=pose` with nested `pose` dict |
| HTTP serves GLB + viewer | **PASS** | `curl -I` → 200, GLB Content-Length 84113904; index.html 200; main.js 200 |
| Manual knee/shoulder controls present | **PASS** | `viewer/main.js` contains knee/shoulder/`modePhysics` |
| Vendor three present (npm optional) | **PASS** | `viewer/vendor/three/three.module.js` |
| GLB path exists | **PASS** | `assets/Xandra.glb` ~81MB |
| Fall scenario code present | **PASS** | `ws_bridge.py --scenario fall` |
| Blender not needed | **PASS** | No `.blend` in tree; runtime used only portable physics + GLB |
| No unresolved imports | **PASS** | Fresh venv imports OK |
| No unexpected external deps | **PASS** | Only pinned requirements (+ mujoco transitive fsspec/typing_extensions/zipp) |
| No open of `/workspace/blender/` during env smoke | **PASS** | Spy on `builtins.open` + DEFAULT_MODEL path check |
| Puppeteer/headless browser capture | **SKIP / N/A** | Chrome present (`/usr/bin/google-chrome-stable`) but `node_modules` excluded; `npm ci` not run in cleanroom. Manual browser WebGL render **not** exercised headlessly here. |
| macOS / Apple Silicon | **NOT TESTED** | Docs only in README_MAC.md |

## Commands (actual)

```bash
CR=/workspace/restore_tests/HumanoidAlpha_v0.2_clean
cp -a /workspace/snapshots/HumanoidAlpha_v0.2_Portable "$CR"
cd "$CR"
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
# Python smoke (sys.path = physics only): HumanoidEnv + HumanoidDiag + extract_full_pose — exit 0
# Fall 50 steps — exit 0
.venv/bin/python -m http.server 8787 --bind 127.0.0.1 &
.venv/bin/python physics/ws_bridge.py --scenario fall --broadcast-hz 30 --port 8765 &
curl -sI http://127.0.0.1:8787/assets/Xandra.glb   # 200, 84113904 bytes
curl -sI http://127.0.0.1:8787/viewer/index.html     # 200
# websockets client recv pose frames — PASS
```

## Limitations

- Clean-room restore validated on **Linux x86_64**, not MacBook Air M4.
- Full browser GLB skinning + Physics WS UI loop not automated (no npm/`node_modules` in portable; headless capture optional).
- Historical absolute paths remain in `reports/*.json` as export FACT; unused at runtime.
- `scripts/01_audit_and_export.py` still references local `Xandra_F4_BodyLab_export_src.blend` (file excluded); re-export needs separate blend — not required for runtime.

## Acceptance vs user criteria

| User criterion | Status |
|----------------|--------|
| GLB loads path exists | PASS (file + HTTP 200) |
| Manual controls code present | PASS |
| Physics WS works | PASS (listen + pose JSON) |
| MuJoCo advances / drives skeleton (bridge smoke) | PASS |
| Fall scenario runs | PASS |
| Blender not needed | PASS |
| No missing files / unresolved imports | PASS |
| No unexpected external deps beyond documented | PASS |
