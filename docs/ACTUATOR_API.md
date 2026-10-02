# Local 33-channel actuator API

The same `physics/control_console.py` process owns MuJoCo, its existing mapper and viewer stream. `--actuator-api` reserves control for the Unix-socket API for the lifetime of that process. The developer console remains connected as a read-only viewer in this mode. Start without that flag to use the console's original interactive ownership and behavior. Stop the previous runtime before changing modes; do not run two runtimes on the same ports.

## Install once on Linux

From this project directory, create a Python 3.13 venv and install the existing dependencies:

```sh
python3.13 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

The current Mac working copy already has its venv. The CLI itself uses only Python's standard library and never imports MuJoCo or loads a model.

## Exact commands

Start the physics process and local HTTP viewer together, in terminal 1:

```sh
./start-actuator-runtime
```

Open http://127.0.0.1:8788/viewer/console.html in the browser on that same machine. In terminal 2, from the same project directory:

```sh
# Exactly 33 values; only channel 0 is nonzero.
./humanoid act 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

# Replace the entire vector, preserving its channel order.
./humanoid act -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

# Submit 33 zero values. Physics continues stepping.
./humanoid zero
```

Stop: press **Ctrl+C in terminal 1**. This stops both services. To restore interactive developer controls on Mac, run the existing `./start-console.command` instead.

For an already managed HTTP service, start only physics:

```sh
.venv/bin/python physics/control_console.py --actuator-api
```

## Contract

- `act` assigns exactly 33 finite numeric values directly to `data.ctrl[:]` in model order. All channels are validated against the loaded MJCF before any is changed. Values are never clamped.
- Each accepted vector persists until another vector or `zero` replaces it. CLI disconnect does not zero or pause the simulation. No pulse duration, controller, position target, balance logic or compensation is added.
- The API runtime starts in dynamic, running mode. Even initial zero controls permit motion from gravity. No pose holding is implied.
- `zero` here means a zero vector; the existing console's **ZERO ALL TORQUES + PAUSE** retains its original meaning in console mode.
- A single physics loop serializes vector application and MuJoCo stepping. CLI calls do not create simulations. An `OK` reply means the vector was applied, not merely queued.
- Concurrent CLI requests are serialized in queue arrival order. There is one API owner of the runtime, not separate owners per short-lived connection.
- Successful stdout is only `OK`. Invalid input or service failure yields a short stderr error and nonzero exit code. No model names, state, anatomy, limits or diagnostic telemetry are returned.
- A timeout/disconnection before the reply leaves the outcome unknown; the CLI does not automatically resend or reset anything.
- Existing emergency handling of nonfinite physics state remains in place; this is not a timed reset of controls.

## Local transport and developer logs

Default socket: `/tmp/humanoid-actuator-<uid>.sock`, mode 0600. Only local processes under the same user can connect. Override consistently in both runtime and CLI with `HUMANOID_ACTUATOR_SOCKET`. No network endpoint is added for actuator commands. Existing HTTP and developer WebSocket stay on localhost.

One request per connection: JSON `{"values":[33 numbers]}` followed by newline; one plain text reply. Socket readers have a 16 KiB bound and a five-second request timeout. Requests enter the existing bounded physics queue.

Developer log: `reports/actuator_commands.jsonl` (override runtime `--actuator-log`). Each request reaching the server records UTC wall time, current simulation time, submitted values and accepted/rejected result. Nonfinite rejected values are stored as strings so the log remains valid JSON. Malformed requests without a readable vector record null. Syntax rejected by the CLI before contacting the runtime cannot appear in the runtime log. The CLI does not read or expose logs.

Clean shutdown removes this process's socket. After a forced kill, an existing socket is deliberately not deleted on startup: first verify that no runtime is listening on it, then remove the stale socket. Never remove a live socket to bypass ownership.

## Tests and verification status

```sh
.venv/bin/python -m unittest discover -s tests -v
```

`test_actuator_api.py` covers strict vector validation, exact model channel ordering, atomic assignment, persistence while stepping, replacement with zero, mapper pose changes, opaque CLI errors, and a real subprocess/Unix-socket/WebSocket integration test. The integration test checks repeated CLI invocations against one running simulation, read-only browser ownership, monotonically advancing simulation time, observer disconnect and invalid transport requests.

Existing control and GLB frame tests are retained. Automated mapper pose checks do not prove anatomical visual correctness. For browser acceptance, run the runtime, submit both vectors, observe the existing Xandra model, confirm selection works and controls are read-only, then stop it and run the original console to check normal editing and double-click reset.

In Codex on this Mac, 14 tests passed. The live transport integration test could not execute because the sandbox rejects binding localhost sockets (`PermissionError: Operation not permitted`). It remains an explicit test failure, not a skipped or claimed pass. Live browser behavior in API mode has not yet been verified. Run the full suite from the regular Terminal or Linux VM before considering end-to-end acceptance complete.

## Files

Modified: `physics/control_console.py` (opt-in API owner, shared queue handling, runtime flags).

Added: `humanoid`, `start-actuator-runtime`, `physics/actuator_runtime.py`, `tests/test_actuator_api.py`, `ACTUATOR_API.md`.

Model, actuator definitions, mapper, calibration, GLB, materials, viewer sources and original launcher are unchanged. Existing tests regenerate their diagnostic JSON reports.
