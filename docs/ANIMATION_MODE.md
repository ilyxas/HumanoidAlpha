# Animation mode (4)

```sh
./START.command animation      # or ./START.command, then 4
```

Opens **http://127.0.0.1:8788/** → `viewer/animation.html`.

- Same screen as Observation: camera presets, LOCK, clock, Screenshot, the two
  on-screen sticks, EXPO/AUTHORITY bar (AUTHORITY disabled, no actuators).
- **No physics.** The launcher starts only the HTTP server: no MuJoCo process,
  no WebSocket, no actuator log.
- Character: the single authoritative `assets/Xandra_Animated.glb` (never
  copied or modified) plus `assets/Xandra_Animated.manifest.json` (loop flags,
  natural ground speeds). If the manifest cannot be read, built-in copies of
  the speeds are used and a warning is shown.

## Clips (EXP002.1-A asset)

Looked up by name (the glTF stores them alphabetically).

| Clip | Duration | Loop | Natural ground speed | Use |
|---|---|---|---|---|
| Idle | 4.50 s | yes | 0 | standing |
| Walk_Fwd | 1.07 s | yes | 1.442 m/s (+Z) | walk |
| Run_Fwd | 0.77 s | yes | 3.016 m/s (+Z) | run |
| Walk_Back | 1.67 s | yes | 0.647 m/s (−Z) | backward walk |
| Squat, Jump, ToPlank, ToBridge | 2.9–4.3 s | no | 0 | debugger preview only (hold last frame) |

The four locomotion clips are in place; the character root is moved in JS.

## Controls

| Input | Action |
|---|---|
| **W / S** | forward / backward (Walk_Back, facing kept) |
| **A / D** | left / right (Xandra turns toward the direction and walks) |
| **Shift** (hold) | run |
| **Caps Lock** | toggle persistent running (also "run lock" in the debugger) |
| **Left stick** (touch / mouse / gamepad) | direction = movement direction, magnitude = speed (EXPO curve); outer 20 % of travel blends walk → run |
| **Right stick** pushed up | forward walk (kept from the first Animation mode) |

Directions are relative to the camera's view on the floor (W = away from
the camera). Keys use physical key codes, so they work on any layout.

All sources produce the same `MoveInput` and go through **one** movement
controller (`viewer/human/locomotion.js`):

- world-space position integrated with delta time; facing = (sin yaw, cos yaw), yaw 0 = +Z (GLB convention);
- target speed = |input| × max speed for the gait (manifest speeds), diagonals never exceed 1;
- acceleration 2.5 m/s², deceleration 3.5 m/s² (release = smooth stop, then Idle);
- backward input (> 120° from forward, hysteresis 100°) keeps the facing and walks back;
- large direction changes turn in place first (no sideways sliding).

## Animation controller

`viewer/animation/anim-controller.js` — state table Idle / Walk / Run (+ Preview):

- state from the actual speed with hysteresis (Idle < 0.05 m/s, Run > 2.1 m/s, back to Walk < 1.9 m/s);
- **playback rate = |ground speed| / natural speed** of the clip, so the in-place cycle and the root travel stay in sync;
- crossfades (0.25–0.30 s): the target clip fades in, the others scale down proportionally (weights always sum to 1);
- a clip that is still visible is never restarted; Walk_Fwd ↔ Run_Fwd are phase-matched by left-foot position (measured at load);
- loop flags from the manifest; unknown clips are treated as one-shot; one-shots use LoopOnce + clampWhenFinished (hold the final pose);
- missing clips fall back gracefully (Run → faster Walk_Fwd, Walk_Back → reversed Walk_Fwd, no Idle → bind pose).

## Animation Debugger (panel under the clock, collapsible)

Live: state, mode, active clips, weights (bars), timeScale per clip, input
source (keyboard / touch / mouse / gamepad), normalized input vector and gait,
controller speed + measured root speed, world position, facing, run modifiers.
Controls: Pause/Resume, playback speed (0.1–2×, scales animation and movement
together), clip selector + Preview, Automatic/Manual mode, camera follow, run lock.
In Manual mode movement input is ignored and the selected clip plays in place.

**Face** section: all morph targets of the head mesh (`F4_Head`, 52 ARKit
names — slots 25/26 are `mouthSmileLeftold` / `mouthSmileRightold`), one slider
each, filter box and **Reset Face**. Body clips have no morph tracks, so the
face is independent of the body animation.

## Camera

The presets are the Observation poses relative to Xandra. With "camera follow"
(default) the orbit target and camera translate with her (orbit/zoom kept);
the floor and grid stay fixed in the world. Floor: 200 × 200 m, 1 m grid;
movement is clamped to ±95 m.

## Tests

```sh
node tests/test_locomotion.mjs      # movement controller, input mapping, state selection
node tests/test_walk_control.mjs    # right-stick mapping (unchanged)
```

Both also run inside `.venv/bin/python -m unittest discover -s tests`.
Validation in the running app (headless Chrome): `reports/exp002_1b/`.
