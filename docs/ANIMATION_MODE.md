# Animation mode (4)

```sh
./START.command animation      # or ./START.command, then 4
```

Opens **http://127.0.0.1:8788/** → `viewer/animation.html`.

- Same screen as Observation (`animation.html` is a copy of `observation.html`):
  camera presets, LOCK, clock, Screenshot, the two on-screen sticks, EXPO/AUTHORITY bar.
- **No physics.** The launcher starts only the HTTP server: no MuJoCo process,
  no WebSocket, no actuator log. Xandra is loaded from `assets/Xandra_Animated.glb`
  and animated by a Three.js `AnimationMixer`.
- Clips in the GLB: `Xandra_Jump` 2.87 s, `Xandra_Squat` 2.93 s, `Xandra_ToBridge` 4.30 s,
  `Xandra_ToPlank` 3.97 s, `Xandra_Walk` 3.20 s (walk cycle). Only the walk is used here.
- The exported walk has root motion (pelvis travels ~4.5 m per cycle). It is made
  in place at load time (linear pelvis X/Z drift removed) so she walks on the spot
  under the fixed cameras. The GLB file itself is not modified.

## Right stick → walk

Vertical axis of the right stick (`+X` on the stick legend; UP is positive for
mouse/touch and for a gamepad via `AXIS_Y_SIGN`):

| Stick | Result |
|---|---|
| centered / inside deadzone 0.08 | no walk |
| pulled down | nothing |
| pushed up | walk, `timeScale = 0.2 + 1.8 · drive` (×0.2 … ×2.0) |
| released | walk weight fades to 0 in 0.35 s → rest pose; clip stops |

`drive` is the deadzone-rescaled value (0…1) shaped by the EXPO buttons
(default **EXPO LINEAR** here, so speed is proportional to stick height).
Pushing again restarts the cycle from frame 0 with a 0.25 s fade-in.
The left stick is displayed but has no effect (no joints without physics);
AUTHORITY is disabled. Mapping code: `viewer/human/walk-control.js`,
test: `node tests/test_walk_control.mjs`.
