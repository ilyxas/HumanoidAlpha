# EXP002.1-B — Animation Controller & Debugger: validation

Validated on 2026-10-08 against the **running app** (`./START.command animation --no-open`,
http://127.0.0.1:8788/ → `viewer/animation.html`). Driver: headless Google Chrome +
puppeteer-core, WebGL via SwiftShader (~6 fps, so per-frame sim dt is capped at 0.1 s and
animation runs at about half real time; all numbers below are in sim time).
Scripts: `scripts/validate_animation_mode.mjs`, `scripts/validate_animation_footslip.mjs`.
Evidence: `validation_evidence.json`, `footslip_hires.json`, `observation_check.json`, PNGs here.
No page errors (only the usual favicon 404).

Asset: `assets/Xandra_Animated.glb` sha256 `0cffd042…97230f` (74 025 548 B) = manifest value.

| # | Check | Result | Evidence |
|---|---|---|---|
| 1 | 8 clips discovered from the final GLB | **pass** | page + debugger list Idle, Walk_Fwd, Run_Fwd, Walk_Back, Squat, Jump, ToPlank, ToBridge; loop flags from manifest (4 loop / 4 once) |
| 2 | Xandra moves through the scene (WASD / left stick) | **pass** | W 4.7 s: z 0 → −6.81 m; touch left stick: x 3.74 → −10.85 m; floor/grid fixed, character root moves; screenshots 02, 04, 09 |
| 3 | Forward, backward, lateral | **pass** | FRONT cam: W → −Z (Walk_Fwd), S → +Z 3.21 m at −0.647 m/s with Walk_Back, facing kept; A → x −5.37 m; D → x +5.22 m; W+D speed 1.442 (no √2) |
| 4 | Shift and Caps Lock | **pass** | Shift: Run, 3.016 m/s, Run_Fwd w=1 ts=1.00; Shift up → Walk 1.442. Caps press → capsLock=true, W alone runs 3.016; second press → false, walk 1.442 |
| 5 | Playback ↔ translation sync | **pass** | every single-clip locomotion frame: \|ts × natural − speed\| ≤ 1e-4 m/s (≈600 frames); root displacement/dt = controller speed (err 0). Planted-foot world speed (dt 0.025 s): Walk_Fwd median 0.8–3.0 cm/s vs body 144 cm/s, Walk_Back 0.2 cm/s vs 64.7 |
| 6 | Release → smooth stop → Idle | **pass** | run 3.016 → 0 in ~0.86 s (3.5 m/s²), walk 1.442 → 0 in ~0.41 s, then Idle crossfade 0.3 s (weights 0.33/0.67/1.0) |
| 7 | No snapping / excessive foot sliding in transitions | **partial** | numbers: per-frame weight change ≤ dt/0.25 s everywhere, weights sum to 1, 0 phase restarts of visible clips, Walk↔Run phase-matched by left foot. Not judged by a human eye at 60 fps (headless 6 fps). Run_Fwd planted-heel (foot_l) slip median 37 cm/s (~12 % of 3 m/s) = asset property (manifest lists foot_l run skate as the largest) |
| 8 | Manual preview incl. one-shots | **pass** | Squat / ToPlank / Jump play once and hold the last frame (time = duration, "holding", weight 1); ToPlank still at 3.967 s 1.5 s later (screenshot 05); Walk_Back loops; movement input ignored in Manual |
| 9 | 52 morph targets accessible & controllable | **pass** | panel "Face · 52 morph targets (F4_Head)", 52 sliders; each slider → influence 0.8 on all 6 head primitives; values persist while walking; Reset Face → all 0 (screenshots 06, 07) |
| 10 | Observation mode + camera/touch interactions | **pass** | Observation (experiment mode) loads, connects, no errors, sticks unchanged (vertically centred), mouse/touch orbit + FRONT preset change the image, its left stick responds to touch. Animation: mouse orbit 4.48 m, touch orbit 4.68 m camera move, presets follow Xandra, touch-drag inside the debugger moves neither camera nor character |

Mobile layout (390×844 and 844×390, touch): sticks low (bottom gap 77 px / 34 px incl.
labels), open debugger overlaps neither stick, the camera buttons nor the EXPO/AUTHORITY bar
(10_*, 11_* screenshots). Real-device safe-area insets (`env(safe-area-inset-*)`, `viewport-fit=cover`)
were not tested on hardware.

Best screenshots: `09_moved_debugger_open.png`, `03_running_shift.png`,
`06_face_inspector_morphs.png`, `05_preview_ToPlank_hold.png`.
