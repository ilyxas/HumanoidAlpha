# Humanoid Alpha v0.2 — MacBook Air M4 setup

**Status:** Packaged and clean-room tested on **Linux** only. **Not tested on macOS** in this packaging run. Steps below are the intended Mac workflow.

## Prerequisites

- macOS on Apple Silicon (M4)
- Python **3.11–3.13** preferred (`python3 --version`). MuJoCo 3.14 wheels are published for many platforms via pip; if `pip install mujoco==3.14.0` fails on your macOS/Python combo, try another 3.11–3.13 minor or check [MuJoCo releases](https://github.com/google-deepmind/mujoco/releases).
- A browser: Safari or Chrome
- Optional: Node.js 18+ only if you want puppeteer capture scripts
- **Blender is NOT required** for runtime

## Extract

```bash
cd ~/Downloads   # or wherever you put the zip
unzip HumanoidAlpha_v0.2_Portable.zip
cd HumanoidAlpha_v0.2_Portable
```

If you received split parts:

```bash
cat HumanoidAlpha_v0.2_Portable.zip.part00 HumanoidAlpha_v0.2_Portable.zip.part01 > HumanoidAlpha_v0.2_Portable.zip
shasum -a 256 HumanoidAlpha_v0.2_Portable.zip   # compare to HumanoidAlpha_v0.2_SHA256.txt
unzip HumanoidAlpha_v0.2_Portable.zip
```

## Python venv + deps

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Pinned (from Linux Body Lab knee_physics venv): `mujoco==3.14.0`, `numpy==2.5.3`, `websockets==17.1`, plus `absl-py`, `etils`, `glfw`, `PyOpenGL`.

## Start runtime (two processes)

**Terminal A — HTTP (must be project root):**

```bash
cd ~/path/to/HumanoidAlpha_v0.2_Portable
source .venv/bin/activate
python -m http.server 8787 --bind 127.0.0.1
```

**Terminal B — MuJoCo WebSocket bridge:**

```bash
cd ~/path/to/HumanoidAlpha_v0.2_Portable
source .venv/bin/activate
python physics/ws_bridge.py --scenario fall --broadcast-hz 30
# scenarios: rest | knee | shoulder | fall | hold | posed
# flags: --host 127.0.0.1 --port 8765
```

Or one-shot helper:

```bash
./scripts/run_demo.sh fall
```

## Browser

Open: http://127.0.0.1:8787/viewer/index.html

1. Wait for `GLB OK | bones=...` in the panel
2. **Manual:** knee / shoulder / shoulderRoll sliders
3. **Physics WS:** click the button; skeleton should track MuJoCo (fall scenario collapses)

## Optional: npm capture scripts

Viewer already vendors Three.js (`viewer/vendor/three`) — **npm is optional**.

```bash
npm ci   # installs puppeteer-core + three per lockfile
# Needs Chrome/Chromium; set executable path in scripts if not at Linux default
export PY="$(pwd)/.venv/bin/python"
node scripts/03_capture_manual.mjs
```

On Mac, puppeteer scripts may hardcode `/usr/bin/google-chrome-stable` — edit or skip if Chrome path differs; manual browser check is enough for POC.

## Smoke checks

```bash
source .venv/bin/activate
python -c "
import sys
from pathlib import Path
sys.path.insert(0, str(Path('physics').resolve()))
from agent_api.env import HumanoidEnv
from bridge_full.mapping_full import extract_full_pose, load_calibration
env = HumanoidEnv()
env.reset()
calib = load_calibration()
pose = extract_full_pose(env.model, env.data, calib)
env.step()
print('OK', env.model.nq, env.model.nu, 'pose_keys', len(pose.get('bones', pose) if isinstance(pose, dict) else []))
"
curl -I http://127.0.0.1:8787/assets/Xandra.glb
curl -I http://127.0.0.1:8787/viewer/index.html
```

## Ports

| Service | Default |
|---------|---------|
| HTTP static | `127.0.0.1:8787` |
| WebSocket bridge | `ws://127.0.0.1:8765` |

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| WS refused / Physics no Hz | Start `ws_bridge.py`; check firewall; confirm `--port 8765` |
| GLB 404 | Start `http.server` from **project root**, not `viewer/` |
| CORS / module errors | Use the local HTTP server (not `file://`); importmap needs HTTP |
| `mujoco` pip fail on Mac | Confirm Python 3.11–3.13 arm64; upgrade pip; check MuJoCo wheel tags |
| Blank canvas / WebGL | Try Chrome; check GPU/WebGL enabled in Safari |
| Wrong Python | `which python` should be `.venv/bin/python` |

## What is NOT included / NOT claimed

- Blender runtime dependency: none
- macOS execution of this package: **not tested** in packaging
- Original Body Lab paths under `/workspace/blender/...`: not required
