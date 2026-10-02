# Control console architecture

## State ownership and command flow

`viewer/console.js` → WebSocket JSON command → bounded 128-command queue → one Python loop in `physics/control_console.py` → MuJoCo → unchanged `mapping_full.py` → pose JSON → `console-viewer.js` → Xandra skin.

WebSocket handlers never mutate qpos, qvel or ctrl. Only the simulation loop calls `Controller.handle`, `Controller.stop` and `Controller.tick`. The first connected client controls the simulation; later clients are read-only. After the controller disconnects, reload a spectator to acquire ownership. Disconnect discards queued commands, zeros torques and pauses on the next loop iteration. Transport ping/pong detects unclean disconnects (10-second ping interval and 10-second timeout); a frozen or lost network connection is not detected instantaneously.

Reset and emergency zero increment a queue epoch and discard earlier pending commands. Later commands belong to the new epoch. Reset uses `mj_resetData`, returns to paused kinematic mode and restores qpos0. Emergency zero also pauses. The queue is bounded; overflow is rejected with an error. Message size is capped at 16 KiB.

Mode changes preserve qpos, zero qvel and ctrl, clear acceleration warmstart, call mj_forward, and enter paused mode. Dynamic simulation advances only after Resume. Position commands are rejected in dynamic mode; torque commands are rejected in kinematic mode. Pause freezes physics while preserving the current torque settings; Zero clears them.

Kinematic hinge/slide commands change only the selected qpos slice. Ball controls use a rotation vector, converted to a normalized quaternion, bounded by the actual model cone angle. Free-root position is separate from its orientation, with an explicit console workspace bound of ±10 m. Ball limits are the model's isotropic cones, not a direction-dependent anatomical range model. Position mode never calls mj_step or advances time.

Dynamic mode assigns actual motor input to ctrl and uses mj_step, followed by mj_forward to synchronize the transmitted transforms with the new qpos. Simulation catches up in fixed model timesteps, caps elapsed time per loop at 50 ms and therefore slows down under sustained load rather than integrating a huge backlog. Non-motor or unlimited actuators remain inventoried but are disabled.

## Protocol

Server sends `inventory` on connection, then the existing `type: pose` envelope at 30 Hz. Pose fields preserve the previous bridge format. `state` adds qpos, qvel, ctrl, mode, paused, revision, contacts, body positions and joint anchors/axes. Commands: `joint`, `torque`, `mode`, `reset`, `pause`, `resume`, `zero`. Each command gets an ack with request ID and revision, or an explicit error.

HTTP binds 127.0.0.1:8788; WebSocket binds 127.0.0.1:8766. Different ports keep the original 8787/8765 demo available. Browser origins are limited to this local console. No authentication is provided for native local clients; this is a local developer tool.

## Explicit rendering correction

The legacy viewer conjugates every absolute bone orientation with `C = Rx(-90°)`:

`q_three = C * q_blender * inverse(C)`

Independent comparison with the actual GLB node hierarchy showed a 1.5708 rad rest discrepancy for **all 23 mapped bones**. The GLB export changes the world basis but retains the bone-local frame. The console uses:

`q_three = C * q_blender`

The maximum discrepancy with the actual GLB rest orientation is below 0.000012 rad. This correction is confined to `console-viewer.js`. The original `main.js`, mapping table, calibration, MJCF and GLB are preserved. This is a documented coordinate-frame correction, not a new anatomical mapping. World quaternions are normalized when deriving local bone orientations.

The viewer applies parents before children and shifts the scene root so the pelvis matches MuJoCo. Joint markers are amber; mapped bone origins and chains are cyan. Orientation error compares the applied visual orientation with the requested orientation. Anchor gap compares the mapped lead bone head with the corresponding physical body origin. Some anchors and derived bones intentionally differ; gaps require interpretation and are not an automatic PASS/FAIL.

## Limitations carried from the model

Clavicles use the existing 40% slerp toward upper arms. Neck interpolation, unmapped twist/corrective bones, skin weights and mesh proportions remain unchanged. Large rotations may reveal skinning defects or physical/visual anchor offsets. Fingers and toes have no independent physical joints in this model. No balance controller is introduced; dynamic mode falls under gravity.
