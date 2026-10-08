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
| Walk_Back | 1.67 s | yes | 0.647 m/s (−Z) | debugger preview only since EXP002.1-C (see below) |
| Jump | 2.87 s | no | 0 | **Space** |
| ToPlank | 3.97 s | no | 0 | **1** (holds the plank 2 s) |
| ToBridge | 4.30 s | no | 0 | **2** (holds the bridge 2 s) |
| Squat | 2.93 s | no | 0 | **3** |

The four locomotion clips are in place; the character root is moved in JS.

**Root vs skeleton (EXP002.1-C §4).** The character group (glTF root) stays at
y = 0 and gets only XZ position and yaw from the movement controller. Pelvis
height and the whole body pose (incl. the jump height and the small baked
pelvis drift of the one-shots) come from the clips' pelvis/bone tracks, which
are not stripped. No root-height compensation, IK, raycasts or physics; the
root is never moved during a one-shot, so no motion is applied twice.

## Controls

| Input | Action |
|---|---|
| **W** | walk away from the camera (camera forward projected on the floor) |
| **S** | toward the camera: she **turns around and walks forward** (Walk_Fwd) |
| **A / D** | camera left / right (she turns toward the direction and walks); diagonals are normalized |
| **Shift** (hold) | run |
| **Caps Lock** | toggle persistent running (also "run lock" in the debugger) |
| **Space** | Jump (one-shot) |
| **1 / 2 / 3** (also numpad) | ToPlank / ToBridge / Squat (one-shots) |
| **Left stick** (touch / mouse / gamepad) | camera-relative direction, magnitude = speed (EXPO curve); outer 20 % of travel blends walk → run |
| **Right stick** pushed up | forward walk (kept from the first Animation mode) |

Directions are relative to the camera's view on the floor. Xandra turns
smoothly (≤ 7 rad/s) to face the movement direction; releasing the keys keeps
her facing. There are no turning keys. Keys use physical key codes, so they
work on any layout.

**Latched movement reference (no feedback loop).** The camera turns behind her
on its own while she moves (see Camera), so reading input against the live
camera would let the camera steer her (holding D would walk a circle). The
reference yaw is therefore *latched*: taken from the camera when input starts,
when a direction key is newly pressed (keys pressed within 0.25 s of the first
form one chord), or when the input source changes; a stick keeps it for the
whole stroke. Releasing one key of a combination keeps it (and a 120 ms grace
ignores the single key left for an instant when a diagonal is released). While
the user actively orbits the camera (mouse/touch drag) the reference follows the
camera live — manual look has priority — and stays latched after the drag.
Camera auto-alignment never changes the reference.

All sources produce the same `MoveInput` and go through **one** movement
controller (`viewer/human/locomotion.js`):

- world-space position integrated with delta time; facing = (sin yaw, cos yaw), yaw 0 = +Z (GLB convention);
- target speed = |input| × max speed for the gait (manifest speeds), diagonals never exceed 1;
- acceleration 2.5 m/s², deceleration 3.5 m/s² (release = smooth stop, then Idle);
- every direction (incl. S) turns her to face it and walks forward; the B-era backward walk
  (> 120°, facing kept, Walk_Back) is still in the code behind `config.backwardWalk` (off);
- large direction changes turn in place first (no sideways sliding);
- while a one-shot owns the body, input is ignored and the root brakes at 8 m/s² (no gliding, no added displacement).

## Animation controller

`viewer/animation/anim-controller.js` — state table Idle / Walk / Run (+ Preview):

- state from the actual speed with hysteresis (Idle < 0.05 m/s, Run > 2.1 m/s, back to Walk < 1.9 m/s);
- **playback rate = |ground speed| / natural speed** of the clip, so the in-place cycle and the root travel stay in sync;
- crossfades (0.25–0.30 s): the target clip fades in, the others scale down proportionally (weights always sum to 1);
- a clip that is still visible is never restarted; Walk_Fwd ↔ Run_Fwd are phase-matched by left-foot position (measured at load);
- loop flags from the manifest; unknown clips are treated as one-shot; one-shots use LoopOnce + clampWhenFinished (hold the final pose);
- missing clips fall back gracefully (Run → faster Walk_Fwd, Walk_Back → reversed Walk_Fwd, no Idle → bind pose).

**One-shots (state Action).** `triggerOneShot(name)` in Automatic mode:

- first come, first served: while a one-shot plays, other one-shot keys (and the same key) are rejected
  with a reason shown in the debugger; keyboard auto-repeat is ignored (`e.repeat`), so holding a key never restarts a clip;
- fade in 0.25 s from standing (0.6 s for a clip that does not start standing, e.g. ToBridge);
- plays once at timeScale 1 (LoopOnce, final pose clamped); clips that end standing (Jump, Squat) return at once,
  ToPlank/ToBridge hold their final pose for 2 s, then return automatically (no stuck state);
  movement input or another one-shot key ends the hold early;
- return: the next locomotion state crossfades in (0.35 s from standing, 0.8 s from plank/bridge);
  movement input is accepted again once the one-shot weight is below 0.5;
- Manual mode (debugger preview) is unchanged and keys do not trigger one-shots there.

Walk_Back remains available in the debugger preview, in the `config.backwardWalk`
path of the movement controller and as the missing-clip fallback logic; WASD and the sticks no longer use it.

## Animation Debugger (panel under the clock, collapsible)

Live: state, mode, active clips, weights (bars), timeScale per clip, input
source (keyboard / touch / mouse / gamepad), normalized input vector and gait,
controller speed + measured root speed, world position, facing, run modifiers,
one-shot (clip, phase, time / hold left, last trigger or rejection reason),
movement reference (latched yaw, why it was latched, live camera yaw) and
camera auto-align (status, azimuth, error, angular velocity).
Controls: Pause/Resume, playback speed (0.1–2×, scales animation and movement
together), clip selector + Preview, Automatic/Manual mode, camera follow, run
lock, camera auto-align.
In Manual mode movement input is ignored and the selected clip plays in place.

**Face** section: all morph targets of the head mesh (`F4_Head`, 52 ARKit
names — slots 25/26 are `mouthSmileLeftold` / `mouthSmileRightold`), one slider
each, filter box and **Reset Face**. Body clips have no morph tracks, so the
face is independent of the body animation.

## Camera

Orbit camera centered on Xandra. With "camera follow" (default) the orbit
target and camera translate with her (orbit/zoom kept); the floor and grid stay
fixed in the world. Floor: 200 × 200 m, 1 m grid; movement is clamped to ±95 m.

- **Stationary:** free orbit (mouse / one-finger touch); orbiting never rotates her.
- **Moving — auto-align:** the camera azimuth is driven toward "behind her"
  (her yaw + 180°) by a critically damped spring (K = 2 /s, ≤ 2.5 rad/s, starts
  from zero angular velocity): no snapping, no overshoot. Only the azimuth
  changes; distance, height and pitch are kept.
- **Manual orbit has priority:** while a drag is active auto-align is off; after
  the drag it fades back in over 1 s.
- **Stop:** auto-align stops (angular velocity decays within ~0.1 s); the camera keeps its orientation.
- LOCK or the "camera auto-align" checkbox turns auto-align off; with follow off it is off too.

**Presets** are relative to her position **and facing** (GLB frame: she faces
+Z, her left is +X): FRONT looks at her face, BACK is behind her, LEFT/RIGHT are
her left/right side, TOP has her facing toward the bottom of the screen, RESET
is the 3/4 view — wherever she is and whichever way she faces.

## Tests

```sh
node tests/test_locomotion.mjs      # movement controller, input mapping, state selection, reference latch
node tests/test_anim_controller.mjs # one-shot arbitration, crossfades (synthetic clips, manifest timing)
node tests/test_walk_control.mjs    # right-stick mapping (unchanged)
```

All also run inside `.venv/bin/python -m unittest discover -s tests`.
Validation in the running app (headless Chrome): `reports/exp002_1b/` (B),
`reports/exp002_1c/` (C, script `scripts/validate_animation_controls.mjs`).
