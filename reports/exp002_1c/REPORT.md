# EXP002.1-C — Third-person controls, orbit camera auto-align, one-shot keys: validation

Validated on 2026-10-08 against the **running app** (`./START.command animation --no-open`,
http://127.0.0.1:8788/ → `viewer/animation.html`), stopped afterwards. Driver: headless Google
Chrome + puppeteer-core, WebGL via SwiftShader (~5.6 fps, so the per-frame sim dt is capped at
0.1 s and the simulation runs at ~0.56× real time; **all times below are sim time**).
Script: `scripts/validate_animation_controls.mjs` (harness copy `/workspace/tools/ha-test/exp002_1c_validate.mjs`).
Evidence here: `validation_evidence.json` (all checks), `validation_evidence_walkjump.json`
(Jump while walking, re-run with a longer hold), `observation_check.json`, `b_regression/`
(the EXP002.1-B script re-run on the new code), PNGs `c01`–`c18`. No page errors (only the usual favicon 404).
Asset untouched: `assets/Xandra_Animated.glb` sha256 `0cffd042…97230f`, manifest unchanged.

## What changed

| File | Change |
|---|---|
| `viewer/human/locomotion.js` | S/backward input now turns her around and walks forward (`backwardWalk: false`; the B backward-walk path is kept behind `config.backwardWalk`). `update(..., { brake })` + `oneShotBrake` 8 m/s². New `createReferenceLatch()`, `yawOf()`, `forwardOfYaw()`. |
| `viewer/animation/anim-controller.js` | One-shot arbitration: `triggerOneShot / releaseOneShot`, state `Action`, play → hold → return with its own fades, rejection reasons, `oneShotPhase`, `actionWeight`, snapshot `oneShot` / `lastOneShot`. Locomotion selection is ignored while a one-shot owns the body. |
| `viewer/animation.js` | Space / 1 / 2 / 3 (and numpad) keys, auto-repeat ignored; latched movement reference wired to the camera; diagonal-release grace; camera auto-align spring; presets relative to her position **and facing**; new readouts in `__animation.state()` and the recorder. |
| `viewer/animation/debugger.js` | Rows One-shot, Move ref, Camera; "camera auto-align" checkbox; panel buttons drop focus after a click (Space = Jump). |
| `tests/test_locomotion.mjs`, `tests/test_anim_controller.mjs` (new), `tests/test_animation_mode.py` | S turn-around, release keeps facing, brake distance, reference latch (incl. a simulated auto-aligning camera: latched = straight line, live camera = circle), chords; one-shot arbitration with the manifest's names/loop flags/durations. |
| `docs/ANIMATION_MODE.md`, `README.md`, `START.command` (help line) | Controls, camera, one-shots, root-vs-skeleton. |
| `scripts/validate_animation_controls.mjs` (new), `scripts/validate_animation_mode.mjs` | C validation; B script: the "touch inside the debugger" probe point now uses the visible panel body (it hit the canvas once the taller panel was scrolled — a harness bug, see b_regression). |

### Movement reference (no feedback loop)
Input is relative to the camera's ground-projected forward, but the reference yaw is **latched**
(`createReferenceLatch`): it is taken from the camera when input starts, when a direction key is
newly pressed (keys pressed within 0.25 s of the first count as one chord), or when the input source
changes; a stick keeps it for the whole stroke. Releasing one key of a combination keeps it, and a
120 ms grace ignores the single key that is left for an instant when a diagonal is released. While
the user is actively orbiting (OrbitControls `start`…`end`, mouse or touch) the reference follows
the camera live — manual look wins — and stays latched afterwards. **Auto-align never writes the
reference**, so the camera swinging behind her cannot steer her (holding D: straight line, below).

### Camera auto-align
Azimuth of the camera around the orbit target is driven toward "behind her" (her yaw + 180°) by a
critically damped spring (K = 2 /s, |ω| ≤ 2.5 rad/s, starts from ω = 0) only while she moves, follow
is on, LOCK is off and no drag is active. Rotation is about the vertical axis through the target, so
distance, height and pitch are unchanged. After a manual drag it fades back in over 1 s; when she
stops, ω decays with 15 /s (≈0.1 s) and the camera keeps its orientation. LOCK or the debugger
checkbox disables it. Stationary: free orbit, never rotates her; the target stays on her.

### One-shot arbitration
First come, first served. Space/1/2/3 (also numpad) call `triggerOneShot` only on a non-repeat
keydown. While a one-shot plays, any other one-shot (or the same key) is rejected with a reason
(`busy: Jump playing`, `ToPlank already holding`, `Jump still fading out`, `manual mode`); the clip is
never restarted. Policy from the manifest poses: fade-in 0.25 s from standing (0.6 s otherwise),
LoopOnce at timeScale 1; Jump/Squat (end standing) return at once with a 0.35 s crossfade;
ToPlank/ToBridge hold the final pose 2 s then return (0.8 s) — movement input or another one-shot
ends the hold early. During play, movement input is ignored and the root brakes at 8 m/s² (no
gliding); input is accepted again once the one-shot weight < 0.5. Manual mode (debugger preview)
is unchanged; keys are rejected there.

### Root vs skeleton (§4)
The glTF root/character group stays at **y = 0** and gets only XZ + yaw from the movement controller;
the pelvis height and the whole pose come from the clips (pelvis tracks are not stripped). No
root-height compensation, IK, raycasts or physics. The root is not moved during a one-shot (measured
XZ drift 0.000 m, root y max 0.000 m) while the pelvis follows the clip (Jump pelvis 0.62 → 1.285 m,
manifest max 1.2885 m; plank pelvis min 0.354 m; bridge 0.122 m), so nothing is applied twice.

## Section 5 verification

| # | Check | Result | Numbers / evidence |
|---|---|---|---|
| 1 | WASD + all 4 diagonals from different camera angles | **pass** | 11 moves from 4 camera setups (camera behind her; on her right with her at yaw 57°; in front of her + real mouse orbit (camera yaw 17.9°, her yaw −126°); RESET 3/4 at yaw 34°): W, S, A, D, W+A, W+D, S+A, S+D. Heading at release vs camera-relative expectation: **0.000° error in all 11**; path direction error ≤ 0.021°; lateral deviation ≤ 1 mm; heading change after release 0.000°; diagonal speed 1.442 m/s (normalized). S: she turns 180° in 0.7 s and walks **forward** with Walk_Fwd; Walk_Back weight > 0 in 0 frames of all automatic tests. `c02`, `c03` |
| 2 | Smooth turning; Idle/Walk/Run transitions | **pass** | max yaw rate 7.0 rad/s (configured cap); per-frame weight change ≤ dt/0.25 s (0.400 at dt 0.1, allowed 0.401) in every recording; weights sum to 1; B regression: 0 phase restarts, playback sync error ≤ 1e-4 m/s, planted-foot speed median 2.6 cm/s at 1.4 m/s (same as B) |
| 3 | Joystick, low and high magnitude | **pass** | touch left stick: 0.365 → 0.526 m/s Walk; 0.703 → 1.013 m/s Walk; rim (1.0) → 3.016 m/s Run (gait 1); diagonal 1.0 → 2.931 m/s; heading error 0.000°, path error ≤ 0.013°, all camera-relative; mouse on the stick works (source "mouse", 1.019 m/s); legacy right stick still walks (B regression). `c06` |
| 4 | Shift and Caps Lock | **pass** | A + Shift: Run 3.016 m/s (Run_Fwd w 1, ts 1.00), Shift up → Walk 1.442; Caps on → S runs 3.016 m/s facing 180° (turned around); Caps off → false. `c07` |
| 5 | Camera orbit + auto-align, incl. manual mouse override | **pass** | Stationary mouse orbit −63°: her yaw change 0.000°, target offset from her 0.000 m, distance 5.573 → 5.573. Hold D (camera starts 90° off): error 76.5° peak → 0.0°, **overshoot 0°, 0 sign changes**, < 5° after 2.5 s, max camera yaw rate 66°/s. S turn-around (camera must go 180°): peak 156° → 0°, overshoot 0°, settled 3.2 s, max 134°/s (< 143°/s cap). Camera distance and height range **0.000 m** in every test. Manual drag while moving: 30 frames "manual orbit", auto ω = 0 during the drag (no fighting, also while the mouse is held still), reference followed the drag; after release "resuming" (7 frames) then error −90° → −1°. Stop: camera yaw drift after stop 0.44°/0.02°. Touch orbit −54°. LOCK: status "off", camera yaw range 0°. `c01`, `c04`, `c05` |
| 6 | Presets after significant translation and rotation | **pass** | Her at (−0.43, −33.16) m, yaw −90°: FRONT/BACK/LEFT/RIGHT/TOP/RESET camera position error **0.000 m**, target error 0.000 m; camera angle in her frame FRONT 0° (face), BACK 180°, LEFT +90°, RIGHT −90°, RESET 38°. (First full run: same result at yaw −133°.) `c08`, `c09`, `c10` |
| 7 | Space / 1 / 2 / 3 trigger the correct clips | **pass** | Space → Jump, 1 → ToPlank, 2 → ToBridge, 3 → Squat, Numpad1 → ToPlank (log + weight 1.0 of exactly that clip). Each key was held with 6 auto-repeat keydowns: one trigger, **0 restarts**. Space in Manual mode → rejected "manual mode". `c11`–`c14` |
| 8 | One-shots complete without loops / stuck states | **pass** | Jump: play 2.8 s (clip 2.867) → return → Idle w 1 (visible 3.3 s). Squat: 2.9 s (2.933) → Idle. ToPlank: 3.9 s + hold 1.9 s → Idle (visible 6.7 s). ToBridge: 4.3 s + hold 1.9 s → Idle. ToPlank hold ended early by W (hold 1.3 s) → Walk 1.442 m/s. During Jump: 3 rejected "busy: Jump playing", W ignored (speed 0, position unchanged). Jump while walking: root stopped in 0.1 s (0.064 m), 0.000 m after that during the jump; W held → walking resumes 3.2 s after the trigger. Root XZ drift 0.000 m and root y 0.000 m in every standing one-shot. Debugger preview of Jump and Walk_Back still works (weight 1, Preview state). `c15` |
| 9 | Observation Mode unchanged | **pass** | no Observation files changed; `./START.command experiment`: Observation loads and connects, no errors, 2 sticks, no animation debugger, mouse/touch orbit and FRONT preset change the image, its left stick responds. `c17`, `c18`, `observation_check.json` |

B regression (`b_regression/`, B script on the new code): all B checks still pass except the
intended change — the B "S_backward" phase now turns her around and walks forward (Walk_Fwd).
Face inspector 52/52 controllable, Manual previews hold the last frame, mobile layouts: the open
debugger overlaps neither stick. Touch-drag inside the debugger: camera delta 0, speed 0.

## Limitations

- Headless SwiftShader at ~5.6 fps with dt capped at 0.1 s; smoothness was judged from numbers, not by
  eye at 60 fps. Gamepad was not tested (no device). Mobile hardware not tested.
- No stand→supine or get-up clips exist (no new animations in scope): ToBridge starts lying, so its entry
  is a 0.6 s blend from standing; returning from plank/bridge to Idle is a 0.8 s crossfade.
- The one-shots' small baked pelvis drift (Jump net 7 cm) is undone by the return crossfade, because the
  root is not moved for one-shots (§4).
- Triggering a one-shot while running brakes the existing motion at 8 m/s² (≈0.57 m from 3.0 m/s at real
  frame rates; 0.064 m measured from a walk at dt 0.1) — deceleration only, no added displacement.
- While the user drags the camera, held input follows the camera live (manual look has priority), so a held
  direction rotates with the drag; after the drag it is latched again.
- A one-shot key pressed during the 0.35–0.8 s return fade of the same clip is rejected ("still fading out").
- Walk_Back is still used by the debugger preview, by `config.backwardWalk` (off) and by the
  missing-clip fallback logic; WASD and the sticks no longer use it.

## Tests

`.venv/bin/python -m unittest discover -s tests`: 31 tests, 30 pass; the one failure is the known
pre-existing `test_collisions.test_original_dynamics_properties_and_actuators_preserved` (also failing
before C). A first run directly after stopping the app also hit one transient port/connection error
(test_launch / test_actuator_api); both pass alone and in two further full runs.
Node: `test_locomotion`, `test_anim_controller`, `test_walk_control`, `test_human_mapper`,
`test_observation_perf` all OK. Report files rewritten by the tests were restored.
