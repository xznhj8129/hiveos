# MPFC
## Multi-Protocol Flight Computer
*Monty Python's Flying Clankers*

MPFC is a lightweight, bus-first runtime for autonomous and remotely controlled robotic systems. A user-written **Program** expresses the purpose of the system. Reusable **plugins** provide capabilities such as UAV control, MAVLink, MSP, CoT/ATAK, tracking, vision, and datalinks.

MPFC deliberately keeps the runtime simple: normal Python processes, MQTT IPC, explicit plugins, and readable Programs. OCCID is the common semantic model inside the runtime.

> The current code still uses the historical names `flight_cores/`, `CoreBase`, and `run_core()`. Conceptually these are Programs. Renaming the code surface is a separate cleanup after the OCCID migration is stable.

## Architecture

```text
                         MPFC runtime

                         Program
                            |
             reusable plugin/API capabilities
                            |
          +-----------------+-----------------+
          |                 |                 |
   uav_controller       atak_interface     tracker/CV
          |                 |                 |
          | OCCID           | OCCID           | OCCID
          v                 v                 v
   +-------------+       CoT/ATAK          MCVST/etc.
   |             |
   v             v
mavsdk_interface msp_interface
   |             |
MAVLink/MSDK     MSP
   |             |
PX4/ArduPilot    INAV/Betaflight
```

The boundaries are intentional:

- **Program** - the main thing being run: take off and land, turn tracker data into flight commands, patrol an area, operate a payload, etc.
- **Plugin** - reusable functional API or external-system interface.
- **`uav_controller`** - stable Program-facing UAV service. It applies reusable vehicle policy/readiness, accepts only immediate UAV OCCID command/input families, and speaks OCCID to Programs and endpoint adapters.
- **Endpoint adapters** - translate OCCID directly to or from native mechanisms such as MAVSDK/MAVLink, MSP, or CoT.
- **MQTT** - local IPC and routing only.
- **OCCID** - semantic model. There is no separate MPFC UAV/ATAK/CV ontology.

A component that translates OCCID into another internal MPFC semantic vocabulary is probably in the wrong place. A component that consumes OCCID, applies real policy or behavior, and emits OCCID is fine.

Higher-level OCCID `Task`, `Plan`, `Assignment`, and `Execution` lifecycle semantics are intentionally not accepted by `uav_controller` merely because they ultimately derive from `Command`. They belong to the OCCID-native MPFC execution ingress that owns local Program selection and high-level execution.

## Program style

Programs should read like intent, not protocol plumbing. For UAV Programs the convenience API in `lib/uav_client.py` constructs and consumes OCCID models:

```python
uav.set_takeoff_altitude(10.0)
uav.arm()
uav.takeoff()
uav.go_to(lat, lon, 20.0)
uav.return_to_launch()
uav.land()
```

Those convenience methods dispatch commands and return request IDs without blocking for an endpoint result. A Program that genuinely needs to wait can explicitly call `uav.execute(command)`.

The same Program-facing API can be backed by PX4/MAVSDK, ArduPilot/MAVSDK, INAV/MSP, or another future adapter without introducing another semantic protocol or pretending that every endpoint has the same request/reply mechanics.

`px4_guide` in `mpfc_additions` is the higher-rate example: tracker state and UAV state are OCCID, guidance math remains in radians, and attitude setpoints go through the UAV service. The MAVSDK adapter alone converts radians to MAVSDK's degree API.

High-rate direct control is intentionally separate from command RPC. A Program begins a direct-control session, publishes latest-value OCCID `Input` samples, then ends the session:

```python
uav.begin_direct_control(occid.DirectControlMode.ATTITUDE_THRUST)
uav.set_attitude(roll_rad, pitch_rad, yaw_rad, thrust)
# ...more latest-value setpoints...
uav.end_direct_control()
```

`ControlAttitudeSetpoint` and `ControlOverride` samples do not receive a request ID or per-sample response.

## OCCID

OCCID is maintained separately in the `occid` repository. MPFC imports the canonical `occid` Python SDK namespace and loads it either from:

1. `OCCID_PATH`, pointing at the OCCID repository root, or
2. a sibling `occid/` checkout beside the MPFC repository.

The generic top-level Python package name `schema` is no longer part of the MPFC consumer boundary.

Transient MQTT payloads render OCCID models directly as JSON-compatible fields, tagged with `_occid_model`, `_occid_model_id`, and `_occid_schema_version` so they remain typed and deterministic while still being readable in ordinary MQTT debugging tools. OCCID's versioned MsgPack `encode()` representation remains the compact binary representation for transports that actually need it. The current semantic schema version is 4.

Representative state streams include:

```text
flight_control
location
attitude
angular_velocity
gnss
autopilot_mission
power
imu
sensor_config
rc_telemetry
remote_control
control_override
control_output
runtime_load
tracker
entity_state
cot_raw
```

These strings are routing keys, not types. The tagged OCCID model inside `data` defines the semantics.

The MSP adapter restores information that was accidentally dropped during the first OCCID cutover rather than treating awkward native data as disposable: receiver bounds, channel maps, mode ranges, RC state, GNSS diagnostics, onboard mission validity/capacity/current waypoint, and selected flight sensor hardware now have explicit OCCID representations. `SetWaypointCommand` also maps to the native MSP waypoint write.

## UAV path

```text
Program
  |
  | UavClient convenience API
  |   commands -> REQUEST
  |   direct-control samples -> INPUT
  v
uav_controller
  |
  | type-gated OCCID in / OCCID out
  v
selected endpoint adapter
  |
  +-- mavsdk_interface -> MAVSDK/MAVLink -> PX4 or ArduPilot
  +-- msp_interface    -> MSP            -> INAV/Betaflight
  +-- liftoff_interface -> simulator telemetry
```

Immediate UAV commands are decomposed by meaning:

- `FlightCommand` - arm, disarm, takeoff, land, RTL, and takeoff-altitude configuration.
- `NavigationCommand` - GoTo, waypoint write, and onboard mission selection.
- `ModeCommand` - mode activation/deactivation. Takeoff, land, and RTL are not encoded as mode changes.
- `DirectControlCommand` - begin/end a portable direct-control session.

High-rate `ControlAttitudeSetpoint` and `ControlOverride` values are OCCID `Input` models rather than command wrappers. Endpoint adapters own the corresponding native lifecycle such as PX4/MAVSDK offboard, MAVSDK manual control, or MSP RC override.

## Reference-frame contract

MPFC follows these conventions unless an OCCID record explicitly states otherwise:

- body frame: FRD
- inertial/world frame: NED
- angular values: radians
- angular velocity: radians/second
- altitude values: meters, positive upward, with explicit datum where needed
- simultaneous absolute and relative altitude observations keep independent vertical datums
- normalized pilot/control axes use signed `-1..+1` semantic control position
- adapters own PWM/MAVSDK/native conversion, including endpoints with reversible throttle/thrust

Frame fields may be optional in OCCID when context is genuinely sufficient, but code performing transforms or control must validate the frames it depends on. `test_takeoff_land` deliberately checks this contract.

CoT `hae` is WGS84 ellipsoid height, not mean-sea-level altitude. CoT/TAK UID is also an external protocol identity, not an OCCID logical subject ID; the ATAK adapter keeps the correlation boundary explicit.

## Plugins

Important current plugins:

| Plugin | Role |
| --- | --- |
| `uav_controller` | Program-facing UAV API/service and reusable flight readiness/policy |
| `mavsdk_interface` | OCCID <-> MAVSDK/MAVLink endpoint adapter for PX4 and ArduPilot |
| `msp_interface` | OCCID <-> MSP endpoint adapter for INAV/Betaflight |
| `liftoff_interface` | OCCID telemetry adapter for Liftoff simulation |
| `atak_interface` | OCCID <-> CoT/ATAK adapter over UDP/TCP |
| `hivelink_interface` | HiveLink datalink integration; semantic OCCID transport work is separate |
| `yolo_detector` | Vision inference plugin |
| `example_hello` | Minimal OCCID request/response runtime smoke test |

`atak_interface` publishes parsed CoT position updates as OCCID `EntityState` and can also expose raw CoT XML as `ProtocolPayload`. Outbound requests can translate OCCID `EntityState`, `HumanTextMessage`, or raw XML `ProtocolPayload` to CoT.

## MQTT bus

Every process is an MQTT peer. The outer envelope is runtime metadata, while `data` stays directly inspectable:

```json
{
  "client": "mavsdk",
  "topic": "uav1/STATE/location",
  "time": 1770835200000,
  "data": {
    "_occid_model": "LocationState",
    "_occid_model_id": 206,
    "_occid_schema_version": [4, 0, 0],
    "inertial_frame": "NED",
    "body_frame": null,
    "position": {
      "_occid_model": "GlobalPosition",
      "_occid_model_id": 196,
      "_occid_schema_version": [4, 0, 0],
      "lat": 45.5017,
      "lon": -73.5673,
      "alt": 37.2,
      "mgrs": null,
      "datum": "WGS84",
      "alt_frame": "SEA_LEVEL"
    },
    "uncertainty": null,
    "attitude": null,
    "altitude": null,
    "velocity": null,
    "navigation_validity": null,
    "gnss": null
  }
}
```

Topics are prefixed with `mpfc/<instance>/`.

Common routing patterns:

| Pattern | Purpose |
| --- | --- |
| `CONTROL/<command>` | runtime lifecycle/control |
| `SET/<client_id>` | runtime parameter mutation |
| `<client>/<ns>/REQUEST` | correlated command/plugin request |
| `<client>/<ns>/RESPONSE` | eventual correlated result |
| `<client>/<ns>/INPUT` | latest-value OCCID input stream, no per-sample response |
| `<client>/<ns>/STATE/<route>` | rate-limited OCCID state stream |
| `DIAG/<client>/<event>` | diagnostics and lifecycle |

Request IDs, response correlation, timeouts, state-rate scheduling, and MQTT envelopes are IPC mechanics. They do not form a second semantic protocol. `uav_controller` forwards backend command results asynchronously rather than blocking its event loop. MSP remains free to use request/reply internally; MAVSDK/MAVLink and streamed controls are not forced into that mechanic.

## Vehicle/backend selection

Vehicle configuration uses OCCID enums:

```yaml
vehicle:
  autopilot: AutopilotType.PX4
  airframe: AirframeType.COPTER
  telem_type: TelemetryType.MAVLINK
```

`main.py` resolves plugin templates, selects exactly one compatible endpoint adapter, injects it into `uav_controller`, and binds the Program to `uav_controller`.

For an INAV/MSP system:

```yaml
vehicle:
  autopilot: AutopilotType.INAV
  airframe: AirframeType.COPTER
  telem_type: TelemetryType.MSP
```

## Examples

Install Python dependencies and ensure Mosquitto is available:

```bash
pip install -r requirements.txt
```

Set `OCCID_PATH` if the OCCID repository is not a sibling of MPFC:

```bash
export OCCID_PATH=/path/to/occid
```

### Sigma Block 1 execution host

The supported Stage 2 path is one Sigma-created `MoveTask`, one MPFC instance
named `uav1`, and one PX4 vehicle. Install only the dependencies needed by this
path with:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements-stage2.txt
export OCCID_PATH=/absolute/path/to/occid
```

Use `flight_cores/execution_host/config.yaml` when PX4 is already running and
sending its companion MAVLink stream to UDP port 14540:

```bash
MAIN_CONFIG=flight_cores/execution_host/config.yaml python main.py
```

That generic host does not arm or take off implicitly. Start with an airborne
vehicle or make an explicit deployment decision to enable the local preparation
policy. The dedicated SITL fixture below opts in safely for its disposable
simulated vehicle.

Use `config_px4_sitl.yaml` to let MPFC start a sibling PX4-Autopilot checkout
and Gazebo. `PX4_AUTOPILOT_PATH` takes precedence over the path in the YAML:

```bash
export PX4_AUTOPILOT_PATH=/absolute/path/to/PX4-Autopilot
MAIN_CONFIG=flight_cores/execution_host/config_px4_sitl.yaml python main.py
```

The execution ingress validates the exact OCCID Execution, Assignment, Plan,
and Task bundle before semantic acceptance. For the acceptance fixture it may
arm and take off a fresh SITL vehicle, then sends `GoToCommand` through
`uav_controller` and the MAVSDK adapter. Completion requires observed
horizontal and datum-correct vertical arrival. Each progress report also
carries the observed `LocationState` as an OCCID `EntityState` for Sigma.

The ingress retains only the latest report for a bounded number of dispatches
in process memory (`report_cache_size`, default 128). Sigma can query a report
by exact Execution and dispatch identity after a Sigma restart. Restarting MPFC
clears this cache, so this is explicit bounded reconciliation, not a durable
distributed log.

Runtime smoke test:

```bash
MAIN_CONFIG=flight_cores/test_core/config.yaml python main.py
```

PX4 SITL acceptance Program:

```bash
MAIN_CONFIG=flight_cores/test_takeoff_land/config_px4.yaml python main.py
```

ArduPilot/MAVSDK acceptance Program:

```bash
MAIN_CONFIG=flight_cores/test_takeoff_land/config.yaml python main.py
```

INAV/MSP telemetry example:

```bash
MAIN_CONFIG=flight_cores/example_msp/config.yaml python main.py
```

Liftoff telemetry example:

```bash
MAIN_CONFIG=flight_cores/example_liftoff/config.yaml python main.py
```

CoT/ATAK example:

```bash
MAIN_CONFIG=flight_cores/atak_example/config.yaml python main.py
```

## Lifecycle and supervision

`main.py` starts the Program and plugins as separate processes and monitors diagnostics.

```text
DIAG/<id>/STARTING
DIAG/<id>/ONLINE
DIAG/<id>/STOPPED
```

A child crash or `DIAG/.../ERROR` triggers coordinated shutdown through `CONTROL/SHUTDOWN`.

Shared runtime machinery lives under `lib/`:

- `common.py` - config, topics, MQTT routing, waits, request correlation
- `core_base.py` - current Program base class
- `plugin_base.py` - plugin lifecycle/response helpers
- `state_scheduler.py` - rate-limited state publishing
- `occid_bus.py` - OCCID packing/unpacking plus command and input helpers
- `occid_topics.py` - local routing keys only
- `uav_client.py` - Program-facing UAV convenience API
- `geo_utils.py` / `reference_frames.py` - geometry/frame helpers

## Development rules

- Put operational meaning in OCCID, not MPFC-local schema files.
- Minimal OCCID means minimum demonstrated coverage, not shallow semantics. Use `deep_ontology` as a semantic reference quarry when a real requirement exposes a missing distinction.
- Programs express intent and policy.
- Reusable capability logic belongs in plugins or small client facades.
- `uav_controller` accepts only its declared immediate UAV command/input families; generic `Command` is not an authorization boundary.
- Endpoint-specific protocol quirks and native lifecycle belong at the endpoint adapter.
- Common semantics do not require common mechanics. Do not force request/reply, streaming, acknowledgements, or native control lifecycle to look identical across protocols.
- MQTT topics route messages; they do not define semantic types.
- Keep frame/unit/datum/native-identity conversions explicit at protocol boundaries.
- Do not create a Task/Execution lifecycle or synchronous RPC for high-rate direct-control samples.
- Add OCCID model coverage when a real Program or adapter demonstrates the need for it, and preserve useful endpoint information rather than discarding it because it is inconvenient to map.

## Status

MPFC is pre-1.0 and evolving quickly. The OCCID migration is intentionally replacing the old internal `protocols/` namespace system rather than maintaining both systems in parallel.
