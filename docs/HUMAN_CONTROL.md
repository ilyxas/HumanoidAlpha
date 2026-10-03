# Human control

Mouse, touch, and gamepad input drive the 33 actuators while the pelvis harness stays an independent wrench. This is not a gait, IK, or balance controller. It does not write `qpos` or `qvel`.

No physical DualShock was attached during validation. Button indices and the Y-axis sign below are assumptions.

## Launch

```sh
./START.command harness
```

That is the existing harness launcher: observation viewer at http://127.0.0.1:8788/ and the control WebSocket at `ws://127.0.0.1:8766`. The clean viewer keeps the white floor, character, clock, and cameras. Two sticks, a one-line gamepad status, `EXPO 2`, and `AUTHORITY 10%` are the only additions. There is no actuator table and no physics HUD.

Do not run `./humanoid act` in a loop for this. The browser sends the vector. Experiment (RAW) and Assisted do not consume it.

## Architecture

```
raw axis -> deadzone 0.08 -> expo (gamma) -> authority -> actuator ctrlrange -> u_cmd[33] -> human_cmd
```

The gamepad module does not know MuJoCo or the socket. The mapper does not know whether a stick is a mouse or a gamepad. Transport only receives a finished 33-vector.

| Piece | File |
| --- | --- |
| Button / axis table, deadzone, authority, rate | `viewer/human/gamepad-map.js` |
| Deadzone, expo curve, ctrlrange map, gamepad read | `viewer/human/normalize.js` |
| Joint cycles and `u[33]` | `viewer/human/joint-mapper.js` |
| Slot A/B edges, per-stick device owner, authority steps | `viewer/human/session.js` |
| JSON frame at 50 Hz | `viewer/human/transport.js` |
| Stick widgets | `viewer/human/sticks-ui.js` |
| Wires the observation page | `viewer/human/attach.js` |
| Observation mount | `viewer/observation.html`, `observation.js`, `observation.css` |
| Runtime accept / stale / ctrl | `physics/control_console.py` |
| Harness wrench (not modified) | `physics/harness.py` |

`physics/harness.py` gains and formula are unchanged (`KP_Z = 30000`). RAW experiment, Assisted, Position mode, actuator ctrl ranges, and the MuJoCo model are unchanged.

## Mapping config

Path: `viewer/human/gamepad-map.js`. Button indices exist only in `GAMEPAD_MAP`.

| Name | Index | Role |
| --- | --- | --- |
| L1 | 4 | cycle slot A (any actuator) |
| R1 | 5 | cycle slot B (any actuator) |
| L2 | 6 | left Z modifier (hold) |
| R2 | 7 | right Z modifier (hold) |
| DpadUp | 12 | authority up one level |
| DpadDown | 13 | authority down one level |
| DpadLeft | 14 | unused |
| DpadRight | 15 | unused |
| axes 0, 1 | left X, left Y | |
| axes 2, 3 | right X, right Y | |

`AXIS_Y_SIGN = -1`. Standard Gamepad Y often reports stick up as `-1`. The sign flip makes stick up `+vertical`, and the mapper treats `+vertical` as a `+X` joint command.

Default deadzone: **0.08**. `abs(x) <= 0.08` becomes 0. Outside, `sign(x) * (abs(x) - 0.08) / 0.92`, so full travel still reaches ±1.

Default authority: **10% (0.10)**. Levels, clamped (no wrap): 5, 10, 20, 40, 60, 80, 100 percent. D-pad Up/Down or the small on-screen buttons. `effective = clip(mapped_ctrl * authority, lo, hi)`.

## Expo

One gamma for both sticks and every joint. It is not a physics change. The runtime stores the already-shaped 33-vector and does not apply a second curve.

Order: raw axis, deadzone and rescale to [-1, +1], `shaped = sign(x) * abs(x)^gamma` (`gamma >= 1`), authority, that actuator's compiled ctrlrange, existing `human_cmd`. For every actuator in this model (0 is inside ctrlrange) that is `torque = actuator_limit * authority * sign(x) * abs(x)^gamma`. `x = 0` is 0. Full stick is still `± actuator_limit * authority` (100% authority is the full ctrlrange; 40% is 40% of it). `gamma = 1` is the old linear map.

On-screen control, next to Authority, live while operating. **EXPO LINEAR** is Expo 0 (`gamma = 1`). Steps: 1, 1.5, 2, 3, 4. Default **EXPO 2** (`gamma = 2`). Top of the range is **EXPO 4** (`gamma = 4`). Example, limit ±60, authority 100%, gamma 2: 0.10 → 0.6, 0.25 → 3.75, 0.50 → 15, 0.75 → 33.75, 1.00 → 60. No smoothing, rate limit, or ramp.

## Sticks

Slot A is the left stick. Slot B is the right stick. There is no left-body vs right-body joint list and no center-joint owner. Each widget is the mouse/touch control and the picture of the normalized stick, whichever device currently owns that stick. The label is the slot and the selected joint (`A · shoulder_l`, `B · shoulder_r`). If a selection is blocked, the label names the slot that already holds it.

Labels, matching the input diagram:

- vertical: **+X up / -X down**
- horizontal: **-Y left / +Y right**
- while Z is held: horizontal drives **Z** instead of Y, and the widget reads `Z ON · horizontal → Z`

On a 3-axis joint (names ending `_x_motor`, `_y_motor`, `_z_motor` on the same joint): vertical maps to X, horizontal to Y, and horizontal maps to Z only while the modifier is held. Releasing L2/R2 or the on-screen Z button stops writing Z on the next command. Z is not latched. The mouse Z control is **hold**, not a toggle, for the same reason.

1-DOF joints expose the one real actuator on the vertical stick. Horizontal and Z do nothing. This model has no XYZ triple for knee, elbow, or each ankle/wrist hinge, so none is invented. Ankle `dp`/`ie` and wrist `flex`/`dev` are separate cycle entries because they are separate joints in the actuator inventory.

Both slots walk one cycle built from the live inventory (`joint`, `name`, `id`, `ctrlrange`). Preferred order, when those joints exist: shoulder_l, elbow_l, wrist_l_flex, wrist_l_dev, hip_l, knee_l, ankle_l_dp, ankle_l_ie, shoulder_r, elbow_r, wrist_r_flex, wrist_r_dev, hip_r, knee_r, ankle_r_dp, ankle_r_ie, lumbar, thoracic, neck. That order is not a side filter. L1 and the left on-screen cycle button advance slot A. R1 and the right on-screen cycle button advance slot B. Either slot may land on any joint in that list, so every actuator in the 33-motor list is reachable from both cyclers, including the opposite limb and the spine.

Conflict ban: the same ctrl index is never selected by both slots. Cycling skips a joint that uses any ctrl index the other slot holds and wraps to the next free one. A direct selection of that joint is refused and the slot stays put. Defaults are already distinct: slot A starts on `shoulder_l`, slot B on `shoulder_r`. Same-frame edges run A then B, so B skips whatever A just took.

A centered stick writes 0 for the axes of the joint that slot is writing. Mouse/touch release springs the dot to center and those commands become 0. Changing the selected joint builds a new full vector, so the previous joint's actuators are 0. A refused change does not clear the joint the slot already holds, and it does not let the other slot's stick write that channel. Unselected actuators are 0. The runtime replaces `ctrl` with that whole vector in harness mode.

## ctrlrange

Normalized input is in `[-1, +1]` after the deadzone, then expo (default gamma 2; gamma 1 is linear). Limits come from the actuator inventory, not from hard-coded torques.

- If `0` is inside `[lo, hi]` (every actuator in this model): negative input maps linearly onto `[lo, 0]`, positive onto `[0, hi]`. Symmetric ranges therefore send `-1 → lo`, `0 → 0`, `+1 → hi`.
- If `0` is outside `[lo, hi]`: `[-1, +1]` maps linearly across `[lo, hi]`. No current actuator does this (`zeroOutsideRange` is empty). It is implemented so a future motor that excludes 0 is not treated as centered.

Then multiply by authority and clip to `[lo, hi]`. The server clips again.

Example from this model's inventory, authority 10%, full +X on `shoulder_l_x_motor` (`ctrlrange [-60, 60]`, actuator id 15): command **6.0**. Full deflection at authority 100% is **60** / **-60**. `hip_r` Y full stick at 10% is **12** because that ctrlrange end is 120. `elbow_l` at 10% is **4** (ctrlrange end 40).

## Arbitration

One owner per side:

1. While the pointer is dragging that side's stick, the mouse owns it.
2. Otherwise, if that side's gamepad stick is outside the deadzone, or L1/L2 (left) or R1/R2 (right) is active, the gamepad owns it.
3. Otherwise the last owner is kept. A centered stick still means zero command. Torque is not latched.

No gamepad: `navigator.getGamepads` missing, throwing, or empty is `GAMEPAD: NOT CONNECTED`. Nothing throws. The device name, when present, is the status element's tooltip only. Axes are not printed in the viewer (`console.log` on connect/disconnect only).

## Transport

The observation page sends one JSON text frame at **50 Hz** (`HUMAN_CMD_HZ`). This is not the MuJoCo timestep (0.002 s). The physics loop uses the latest vector.

```json
{"op":"human_cmd","u":[33 finite numbers],"t":1710000000000}
```

- `op` is exactly `human_cmd`
- `u` is a complete actuator command, index = MuJoCo actuator id, length 33
- `t` is client milliseconds (`Date.now()`). The runtime does **not** use `t` for the stale check. It stores its own monotonic receive time so a frozen tab cannot keep torque alive by stamping a fresh `t`.

The handler only stores the latest object (`stage_human_cmd`). It does not write MuJoCo state and it does not enter the 128-command console queue, so a 50 Hz stream cannot delay RAW actuator-API commands. The physics loop calls `ingest_human_cmd` and then `poll_human_stale`.

`ingest_human_cmd` is a no-op unless the harness is enabled. Experiment, debug, and assisted ignore the message and leave `ctrl` alone. When the harness is on, the vector is clipped to each actuator's real `ctrlrange` and copied to `human_u`, `u_cmd`, and `data.ctrl`.

On each dynamic tick, if a human stream has been accepted, `ctrl` is set from `human_u` and then `HarnessSupport.apply` writes `xfrc_applied` on the pelvis. The wrench does not read `ctrl`. After the first accepted `human_cmd`, a later `humanoid act` is overwritten on the next tick. Do not mix them in harness mode.

## Stale input

`HUMAN_CMD_STALE_S = 0.200` (inside 150–250 ms). If the harness human stream is quiet longer than that, `human_u` and `ctrl` go to 0. The harness stays enabled and keeps applying `xfrc`. Disconnect, socket close, and a frozen tab are the same case: no new message, so the timeout fires. A centered stick is already a zero vector and does not wait for the timeout.

## Tests

No gamepad hardware.

```sh
node tests/test_human_mapper.mjs
.venv/bin/python -m unittest tests/test_human_control.py -v
```

The Python test also runs the node file and writes `reports/human_control/validation.json`.

Mouse/logic results (all pass):

| Check | Result |
| --- | --- |
| 1 slots A and B independent | pass. shoulder_l X at 10% = 6, hip_r Y at 10% = 12, the other slot's actuators stay 0 |
| 2 release springs to 0 | pass. full `u` is 33 zeros |
| 3 joint switch clears previous | pass. shoulder_l X returns to 0, elbow_l = 4 |
| 4 Z release clears Z | pass. shoulder_l Z = 6 while held, 0 after release; Y comes back. Elbow stays the single real actuator |
| 5 deadzone 0.08 | pass. ±0.08 → 0, ±1 → ±1, input 0.54 → 0.5 |
| 6 authority | pass. default 0.10, D-pad up → 0.20 once, clamp at 0.05 and 1 |
| 7 full deflection vs real ctrlrange | pass. shoulder_l X → 60 at 100% and 6 at 10% (`hi = 60`). Z full negative → -60. `zeroOutsideRange` empty |
| 8 33-vector reaches runtime `ctrl` | pass. id 15 = 6.0 while harness steps |
| 9 harness stays independent | pass. peak `|xfrc_applied[pelvis]|` = 1097.65 after 400 steps, harness still enabled, `KP_Z` still 30000 |
| 10 stale timeout | pass. at 0.201 s past the last message, `ctrl` is 0; next tick keeps 0; pelvis wrench still nonzero; harness still on |
| 11 RAW / assisted | pass. dynamic tick without `human_cmd` advances time; ingest while harness is off does not change `ctrl`; torque op still sets ctrl; assisted mode still constructs `AssistedStandController` |
| 12 no gamepad | pass. missing API, throw, and empty list do not throw |
| 13 one config object | pass. indices only in `viewer/human/gamepad-map.js` |
| 14 no qpos/qvel on the human path | pass. `viewer/human/*.js` and `ingest` / `poll` / `stage` do not assign generalized coordinates. Ingest leaves `qpos` and `qvel` unchanged |
| slots A/B, any actuator, no shared ctrl index | pass. both cyclers reach every joint; selecting the other slot's joint is skipped or refused; lumbar is not double-written; leaving it zeros that ctrl |

`gamepad_hardware_validated` is false.

## Assumptions that need a real DualShock

- Button indices match the W3C Standard Gamepad map (L1=4 … D-pad=12–15). Bluetooth, Steam Input, and some OS mappings differ.
- `AXIS_Y_SIGN = -1` (stick up is raw `-1`). If a pad reports up as `+1`, flip this constant in `GAMEPAD_MAP` only.
- Trigger “pressed” is `button.pressed` or `value > 0.5`.
- Ankle and wrist are cycled as separate 1-DOF hinges, vertical stick only.
- Authority does not wrap past 5% or 100%.
