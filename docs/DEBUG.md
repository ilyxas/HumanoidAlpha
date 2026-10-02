# Humanoid Alpha — Full Body Control Console

Local developer console for the existing MuJoCo → mapper → Three.js → Xandra pipeline.

## Launch on macOS

From this folder:

```bash
./start-console.command
```

The launcher uses this copy's `.venv`, installs `requirements.txt` if imports are missing, starts HTTP on **8788** and control WebSocket on **8766**, and opens:

http://127.0.0.1:8788/viewer/console.html

Keep the terminal open; **Ctrl+C** stops both services. The original demo's ports 8787/8765 are separate. If either console port is occupied, the launcher stops without killing unrelated processes.

For a clean installation use Python 3.11–3.13 on Apple Silicon:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
./start-console.command
```

Three.js is included. Blender and npm are not required.

## Use

- **Position:** select any discovered joint by anatomical region. Hinge sliders use model limits. Ball joints use a rotation vector bounded by their model cone. Root translation and orientation have separate controls. Time remains frozen. Edits that cross a collision surface are rejected along the movement path; the last accepted pose is held. Move away from the obstacle or adjust another joint first.
- **Torque:** select an actuator, enter a bounded motor command, then **Resume**. Gravity and floor contacts are active. Commands are motor inputs, not position targets.
- **Pause:** freezes physics and retains command values. **ZERO ALL TORQUES + PAUSE:** clears commands and stops time. **Reset reference:** restores qpos0, zero velocity/torques, paused Position mode.
- Select a joint to inspect physical qpos/qvel, body, mapped bones, orientation error and anchor separation. Amber indicates the physical joint; cyan indicates visual bone origins and chains.
- **Frame character**, orbit/zoom/pan, SkeletonHelper, physical links and Screenshot are available.
- The first connected tab controls the simulation. Additional tabs are read-only. Close the controlling tab, then reload the desired tab to transfer control. Disconnect zeros torques and pauses when detected.

Every discovered physical joint and actuator is enumerated from the loaded MuJoCo model. The current inventory contains **20 joints including the free root, 33 actuators, 47 qpos values and 39 qvel values**. Fingers/toes and clavicles are not independent physical joints in this model.

## Evidence and caveats

- `reports/control_inventory.json`: actual model inventory.
- `reports/control_joint_sweep.json`: independent qpos changes and mapper response; this alone is **not** anatomical validation.
- `reports/glb_frame_validation.json`: comparison with exported GLB rest frames.
- `reports/CONTROL_VALIDATION.md`: test results, visual evidence and remaining limits.
- `CONTROL_ARCHITECTURE.md`: command flow, state ownership, mode transitions and the explicit visual frame correction.

The GLB is unchanged. Collision surfaces now follow convex envelopes of Xandra's rest skin, with explicit contacts away from adjacent joint seams. Visual clavicles follow the welded physical clavicles. Masses, inertias, joints, actuator limits/order and the physics timestep are preserved. See `COLLISIONS.md` for scope, verification and approximation limits. The console also fixes a legacy 90° absolute-bone coordinate conversion error, documented in the architecture note. Skinning/twist/corrective defects are not automatically repaired. Low orientation error alone does not establish anatomical correctness.

## Tests

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest discover -s tests -v
```

Original runtime documentation remains in `README_MAC.md`, and original viewer and `physics/ws_bridge.py` remain available via the original `scripts/run_demo.sh`.

## Body picking and slider defaults

Click a visible part of Xandra to select its physical control and open/scroll to the anatomical group on the left. Picking uses the currently deformed skin and its bone weights, so it follows the posed character. Fingers and twist bones resolve through their rig ancestors. When several controls share a body (wrist/ankle axes or torque motors), the current matching control is retained; otherwise the first matching control is selected. Other axes remain in the same group. Dragging to orbit or pan does not select a body part.

Double-click a right-panel slider (or double-tap on touch input) to restore that component's reference value. Joint defaults come from the model's qpos0; torque defaults are zero. Root height resets to its model reference height, not zero. Resetting one rotation-vector component preserves the others. Refresh the console page to load these UI changes; no server restart is required for this existing model.
