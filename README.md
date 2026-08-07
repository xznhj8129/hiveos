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
- **`uav_controller`** - stable Program-facing UAV service. It applies reusable vehicle policy/readiness and speaks OCCID to Programs and endpoint adapters.
- **Endpoint adapters** - translate OCCID directly to or from native mechanisms such as MAVSDK/MAVLink, MSP, or CoT.
- **MQTT** - local IPC and routing only.
- **OCCID** - semantic model. There is no separate MPFC UAV/ATAK/CV ontology.

A component that translates OCCID into another internal MPFC semantic vocabulary is probably in the wrong place. A component that consumes OCCID, applies real policy or behavior, and emits OCCID is fine.

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

The same Program-facing API can be backed by PX4/MAVSDK, ArduPilot/MAVSDK, INAV/MSP, or another future adapter without introducing another semantic protocol.

`px4_guide` in `mpfc_additions` is the higher-rate example: tracker state and UAV state are OCCID, guidance math remains in radians, and attitude setpoints go through the UAV service. The MAVSDK adapter alone converts radians to MAVSDK's degree API.

## OCCID

OCCID is maintained separately in the `occid` repository. MPFC loads its generated Pydantic package either from:

1. `OCCID_PATH`, pointing at the OCCID repository root, or
2. a sibling `occid/` checkout beside the MPFC repository.

Transient MQTT payloads use OCCID's own versioned MsgPack `encode()` representation, base64-wrapped only because the MQTT client currently carries JSON envelopes. MPFC does not invent another durable object representation.

Representative state streams include:

```text
flight_control
location
attitude
angular_velocity
gnss
power
imu
rc_telemetry
control_override
control_output
runtime_load
tracker
entity_state
cot_raw
```

These strings are routing keys, not types. The model encoded inside the payload defines the semantics.

## UAV path

```text
Program
  |
  | UavClient convenience API -> OCCID commands/state
  v
uav_controller
  |
  | OCCID in / OCCID out
  v
selected endpoint adapter
  |
  +-- mavsdk_interface -> MAVSDK/MAVLink -> PX4 or ArduPilot
  +-- msp_interface    -> MSP            -> INAV/Betaflight
  +-- liftoff_interface -> simulator telemetry
```

Examples of low-level OCCID commands include `ArmCommand`, `TakeoffCommand`, `GoToCommand`, `ReturnToLaunchCommand`, `SetControlAttitudeCommand`, and `SetControlOverrideCommand`. These are immediate control imperatives, not Task/Assignment/Execution lifecycle objects.

## Reference-frame contract

MPFC follows these conventions unless an OCCID record explicitly states otherwise:

- body frame: FRD
- inertial/world frame: NED
- angular values: radians
- angular velocity: radians/second
- altitude values: meters, positive upward, with explicit datum where needed
- normalized control axes are protocol-independent; adapters own PWM/MAVSDK/native conversion

Frame fields may be optional in OCCID when context is genuinely sufficient, but code performing transforms or control must validate the frames it depends on. `test_takeoff_land` deliberately checks this contract.

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

Every process is an MQTT peer. The outer envelope is runtime metadata:

```json
{
  "client": "sender_id",
  "topic": "runtime/topic",
  "time": 1770835200000,
  "data": {}
}
```

Topics are prefixed with `mpfc/<instance>/`.

Common routing patterns:

| Pattern | Purpose |
| --- | --- |
| `CONTROL/<command>` | runtime lifecycle/control |
| `SET/<client_id>` | runtime parameter mutation |
| `<client>/<ns>/REQUEST` | correlated plugin request |
| `<client>/<ns>/RESPONSE` | correlated plugin result |
| `<client>/<ns>/STATE/<route>` | rate-limited OCCID state stream |
| `DIAG/<client>/<event>` | diagnostics and lifecycle |

Request IDs, response correlation, timeouts, state-rate scheduling, and MQTT envelopes are IPC mechanics. They do not form a second semantic protocol.

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
- `occid_bus.py` - OCCID packing/unpacking and request helpers
- `occid_topics.py` - local routing keys only
- `uav_client.py` - Program-facing UAV convenience API
- `geo_utils.py` / `reference_frames.py` - geometry/frame helpers

## Development rules

- Put operational meaning in OCCID, not MPFC-local schema files.
- Programs express intent and policy.
- Reusable capability logic belongs in plugins or small client facades.
- Endpoint-specific protocol quirks belong at the endpoint adapter.
- MQTT topics route messages; they do not define semantic types.
- Keep frame/unit conversions explicit at protocol boundaries.
- Do not create a Task/Execution lifecycle for high-rate low-level control samples.
- Add OCCID model coverage when a real Program or adapter demonstrates the need for it.

## Status

MPFC is pre-1.0 and evolving quickly. The OCCID migration is intentionally replacing the old internal `protocols/` namespace system rather than maintaining both systems in parallel.
