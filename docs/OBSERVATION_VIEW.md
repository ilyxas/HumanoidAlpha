# Observation view

Start with `./start-actuator-runtime`, then open:

- Observation: http://127.0.0.1:8788/viewer/observation.html
- Existing lab: http://127.0.0.1:8788/viewer/console.html

The actuator runtime, CLI and ownership rules are unchanged. In actuator mode the lab remains a read-only diagnostic observer, as before. Its normal interactive launcher is unchanged.

## Observer controls

Drag to orbit, right-drag to pan, scroll/pinch to zoom. LOCK disables OrbitControls pointer input; UNLOCK restores it. Presets remain available while locked. The clock uses browser-local wall time, HH:MM:SS. Screenshot generates a PNG containing the same scene and camera/clock overlay shown on screen, with local date and time in its filename.

No inventory, physical state, skeleton, selection, diagnostics or lab controls are placed in this page's DOM. Clicking the character only participates in camera interaction. The viewer never sends WebSocket commands. It consumes pose messages from the existing stream and discards the other message types and state fields. The wire protocol still contains developer data: this UI separation is not a security boundary against DevTools/network inspection or access to the separate lab URL.

## Fixed perspective camera

Three.js Y-up coordinates, perspective FOV 40°, near .05, far 50. All presets target `(0, .7, 0)` and are independent of the current body pose. FRONT is viewed from +Z; LEFT from +X; BACK from -Z; RIGHT from -X. These are fixed world directions and do not follow the character's facing direction.

| Preset | Position | Up |
| --- | --- | --- |
| FRONT | (0, 1.6, 5.5) | (0, 1, 0) |
| LEFT | (5.5, 1.6, 0) | (0, 1, 0) |
| BACK | (0, 1.6, -5.5) | (0, 1, 0) |
| RIGHT | (-5.5, 1.6, 0) | (0, 1, 0) |
| TOP | (0, 6.2, 0) | (0, 0, -1) |
| RESET / initial | (3.6, 2.5, 4.6) | (0, 1, 0) |

OrbitControls uses a tiny polar-angle epsilon at the exact top pole. Repeated presets produce the same camera view. Camera RESET does not send a physics reset. Resizing changes aspect ratio only. No tracking, automatic reframing or pose classification is implemented.

The opaque matte floor lies at Y=0, matching the physical floor's height after the existing coordinate conversion. Its material, sparse grid, lighting and shadows are visual only. Xandra's GLB and materials are not edited.

## Developer performance measurements

Open `viewer/observation.html?profile=1` and inspect browser developer Console. Every ~5 seconds `[observation-perf]` reports actual elapsed window seconds and:

- `framesPerSecond`: animation callbacks divided by elapsed time.
- `poseMessagesPerSecond`: only received pose messages, divided by elapsed time.
- `frameIntervalMs`: callback intervals, including stalls.
- `poseHandlerCpuMs`: JSON parse plus pose handling.
- `poseApplyCpuMs`: the existing pose-to-bone application work.
- `modelWorldMatricesCpuMs`: outermost model world-matrix updates, including nested descendants; nested calls are not counted twice in this bucket.
- `skinPaletteCpuMs`: CPU skeleton palette update calls (unique skeletons wrapped once).
- `rendererCpuMs`: synchronous renderer.render duration, including its own matrix/skinning work and shadow passes.
- `frameWorkCpuMs`: controls update and renderer submission in the animation callback.
- `longFramesOver50ms`: callback gaps above 50 ms.
- `estimatedMissed60Hz`: approximate missed 60 Hz frame opportunities; not an exact count of dropped GPU/display frames and not calibrated to other refresh rates.

Timing distributions include count, mean, p95 and maximum. Categories overlap; do not add them together. World-matrix and palette timings may occur inside both pose application and rendering. GPU skinning/drawing/compositor completion is asynchronous and is NOT measured by these CPU timers. Use the browser's GPU/performance tooling for GPU timing. No artificial gl.finish or timing/stepping changes are added.

`[observation-event]` records model load start/completion/error and connection lifecycle. Reloading emits another load sequence. No model names or physical state are logged. Profiling is off by default and does not add a visible panel. It has measurement overhead when enabled. Measurements made on this Mac do not diagnose the separate Linux VM's ~5 FPS; collect that VM's logs with the same flag first.

Optional `wsPort` selects an alternate local test port; the normal port remains 8766. A test runtime must separately allow the test HTTP origin. Production WebSocket configuration is unchanged.

## Validation

- All 15 Python tests passed, including real repeated CLI calls to one physics process.
- `node tests/test_observation_perf.mjs` passed: actual-time rates, nested matrix timing, disabled instrumentation.
- Browser checks: live CLI motion; clean DOM and image; opaque floor/shadows; orbit and zoom; locked drag and scroll leave static scene pixels identical; all six repeated presets give identical static scene pixels; clock advances; lab receives live state and retains its inventory and controls.
- Original physics, API/CLI, model, GLB, mapper, calibration and lab viewer files matched their pre-change SHA-256 values.
- Touch gesture blocking follows the existing OrbitControls enabled guard; physical touchscreen testing was not available.
- Screenshot export code runs without a browser error, but the in-app browser did not return a download event/file. Native PNG download remains to be checked in the target Linux browser. Saved browser screenshots in `reports/observation/` are QA evidence, not proof of native download completion.

Modified: `start-actuator-runtime` (printed viewer URL only).
Added: `viewer/observation.html`, `viewer/observation.css`, `viewer/observation.js`, `viewer/observation-perf.js`, `tests/test_observation_perf.mjs`, this document and QA evidence in `reports/observation/`.
