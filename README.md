# MPFC
## Multi-Protocol Flight Computer
*Monty Python's Flying Clankers*

MPFC is a lightweight companion-computer runtime for autonomous and remotely controlled robotic systems.

A user-written **Program** expresses what the system should do. Reusable **plugins** provide capabilities such as UAV control, MAVLink, MSP, CoT/ATAK, tracking, vision, and datalinks. **OCCID** is the common semantic model used between those components, while MQTT provides local IPC and routing.

MPFC deliberately avoids turning the companion computer into a large framework. The runtime is normal Python processes, explicit configuration, readable Programs, and small adapters around real external systems.

The current source still uses the historical names `flight_cores/`, `CoreBase`, and `run_core()` for Programs. Those names predate the current architecture; renaming them is separate cleanup, not a second runtime path.

## Architecture

```text
                         MPFC runtime

                         Program
                            |
               reusable capability APIs
                            |
          +-----------------+------------------+
          |                 |                  |
   uav_controller       atak_interface      tracker/CV
          |                 |                  |
          | OCCID           | OCCID            | OCCID
          v                 v                  v
   +-------------+       CoT/ATAK           MCVST/etc.
   |             |
   v             v
mavsdk_interface msp_interface
   |             |
MAVSDK/MAVLink   MSP
   |             |
PX4/ArduPilot    INAV/Betaflight
```

The boundaries are intentional:

- **Program**: the main behavior being run, for example patrol an area, track a target, operate a payload, or execute assigned work.
- **Plugin**: reusable capability logic or an interface to an external system.
- **`uav_controller`**: the stable Program-facing UAV service. It owns reusable vehicle readiness and command policy and exposes OCCID operations to Programs.
- **Endpoint adapters**: translate OCCID to and from native mechanisms such as MAVSDK/MAVLink, MSP, or CoT.
- **Execution ingress**: accepts higher-level OCCID execution records and delegates concrete local behavior through the normal Program and UAV interfaces.
- **MQTT**: local IPC and routing. MQTT topics are not the semantic model.
- **OCCID**: operational meaning shared across MPFC components and external systems.
- **HiveLink**: optional datalink integration for moving traffic across constrained or heterogeneous links. MPFC does not depend on HiveLink for its local runtime semantics.

A component that merely translates OCCID into another MPFC-specific semantic vocabulary is probably in the wrong place. Components may consume OCCID, apply real policy or behavior, and emit OCCID without inventing a second ontology.

## Project dependencies

At the source-project level MPFC is designed to live beside only two related repositories:

```text
parent/
  mpfc/
  occid/
  hivelink/
```

- **OCCID** provides the canonical Python SDK and operational data model.
- **HiveLink** provides the optional datalink implementation used by `hivelink_interface`.
- Everything else is an ordinary Python or operating-system package dependency declared by the projects.

MPFC loads OCCID from either:

1. `OCCID_PATH`, pointing at the OCCID repository root, or
2. a sibling `occid/` checkout beside MPFC.

The Raspberry Pi appliance also installs the sibling HiveLink checkout directly so its filesystem contains the same three project trees used during development.

## Program style

Programs should read like intent, not protocol plumbing. The UAV convenience API in `lib/uav_client.py` constructs and consumes OCCID models:

```python
uav.set_takeoff_altitude(10.0)
uav.arm()
uav.takeoff()
uav.go_to(lat, lon, 20.0)
uav.return_to_launch()
uav.land()
```

Those convenience methods are ordinary MPFC SDK helpers. They now construct the generic OCCID Control commands rather than API-shaped OCCID classes: arming is a `StateChangeCommand`, takeoff-altitude setup is a `ConfigurationCommand`, takeoff/land/RTL are `ProcessControlCommand`s, and navigation is a `MotionCommand`. They dispatch and return request IDs without blocking for endpoint completion. A Program that actually needs to wait for an endpoint result can explicitly call `uav.execute(command)`.

The same Program-facing API can be backed by PX4/MAVSDK, ArduPilot/MAVSDK, INAV/MSP, or another future adapter without introducing another semantic protocol or pretending that every endpoint has identical mechanics.

High-rate direct control is separate from command RPC. A Program starts an adapter-local control process, publishes latest-value OCCID `Input` samples, then releases the process:

```python
uav.begin_direct_control("ATTITUDE_THRUST")
uav.set_attitude(roll_rad, pitch_rad, yaw_rad, thrust)
# publish more latest-value setpoints
uav.end_direct_control()
```

`ControlAttitudeSetpoint` and `ControlOverride` samples do not receive a request ID or one response per sample. The process lifecycle is expressed with ordinary `ProcessControlCommand`; no direct-control-specific OCCID Command hierarchy exists.

## OCCID inside MPFC

Transient MQTT payloads render OCCID models directly as JSON-compatible fields tagged with:

```text
_occid_model
_occid_model_id
_occid_schema_version
```

This keeps the local bus inspectable with ordinary MQTT tools while retaining deterministic type identity. OCCID's versioned MsgPack `encode()` representation remains available for transports that need a compact binary representation.

Representative state routes include:

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

These strings are routing keys, not types. The OCCID model inside `data` defines the semantics.

The MSP adapter intentionally preserves awkward native information rather than discarding it because it is inconvenient to map. Receiver bounds, channel maps, mode ranges, RC state, GNSS diagnostics, onboard mission state, selected sensor hardware, and related endpoint details have explicit OCCID representations where MPFC currently needs them.

## UAV path

```text
Program
  |
  | UavClient
  |   commands -> REQUEST
  |   direct-control samples -> INPUT
  v
uav_controller
  |
  | OCCID in / OCCID out
  v
selected endpoint adapter
  |
  +-- mavsdk_interface  -> MAVSDK/MAVLink -> PX4 or ArduPilot
  +-- msp_interface     -> MSP            -> INAV or Betaflight
  +-- liftoff_interface -> simulator telemetry
```

Immediate UAV operations use OCCID's semantic Command families:

- `StateChangeCommand`: arming and supported state/mode changes.
- `ProcessControlCommand`: takeoff, land, RTL, and adapter-local direct-control process lifecycle.
- `ConfigurationCommand`: supported endpoint configuration such as takeoff altitude.
- `MotionCommand`: MOVE_TO and supported motion-control operations.
- `ResourceCommand` and `ExecutionCommand`: accepted at the generic UAV boundary but rejected by an endpoint unless that endpoint has an explicit implementation.

High-rate `ControlAttitudeSetpoint` and `ControlOverride` values are OCCID `Input` models rather than command wrappers. Endpoint adapters own native control lifecycle such as PX4/MAVSDK offboard mode, MAVSDK manual control, or MSP RC override.

Higher-level OCCID `Task`, `Plan`, `Assignment`, and `Execution` records do not go through `uav_controller` merely because they eventually produce vehicle commands. The execution ingress owns that higher-level lifecycle and delegates immediate UAV work through `uav_controller`.

For the current Block 1 handler, MPFC accepts `TaskManeuver` with `ManeuverIntent.MOVE`. The task preserves its required natural-language `instruction`; its destination is resolved through exactly one `location_ref` to a Location record carrying a `GlobalPosition`. Unsupported Task families or intents are rejected during semantic acceptance rather than guessed into endpoint behavior.

## Reference-frame contract

MPFC follows these conventions unless an OCCID record explicitly states otherwise:

- body frame: FRD
- inertial/world frame: NED
- angular values: radians
- angular velocity: radians per second
- altitude values: meters, positive upward, with an explicit datum where needed
- simultaneous absolute and relative altitude observations retain independent vertical datums
- normalized pilot/control axes use signed `-1..+1` semantic control position
- endpoint adapters own PWM, MAVSDK, protocol, and unit conversion

Frame fields may be optional in OCCID when context is genuinely sufficient, but code performing transforms or control must validate the frames it depends on.

CoT `hae` is WGS84 ellipsoid height, not mean-sea-level altitude. CoT/TAK UID is also an external protocol identity, not an OCCID logical subject ID. The ATAK adapter keeps those boundaries explicit.

## Plugins

Important current plugins include:

| Plugin | Role |
| --- | --- |
| `uav_controller` | Program-facing UAV service and reusable flight readiness/policy |
| `execution_ingress` | OCCID execution validation, dispatch, progress, and result reporting |
| `mavsdk_interface` | OCCID to/from MAVSDK/MAVLink for PX4 and ArduPilot |
| `msp_interface` | OCCID to/from MSP for INAV and Betaflight |
| `liftoff_interface` | simulator telemetry adapter |
| `atak_interface` | OCCID to/from CoT/ATAK over UDP/TCP |
| `hivelink_interface` | optional HiveLink datalink bridge |
| `yolo_detector` | vision inference plugin |
| `example_hello` | minimal OCCID request/response example |

`atak_interface` publishes parsed CoT position updates as OCCID `EntityState` and can expose raw CoT XML as `ProtocolPayload`. Outbound traffic can translate supported OCCID records back to CoT.

`hivelink_interface` bridges canonical OCCID models between HiveLink and the node-local `OCCID/IN` / `OCCID/OUT` topics. HiveLink remains transport; it does not know execution semantics.

## MQTT bus

Every MPFC process is an MQTT peer. Topics are prefixed with:

```text
mpfc/<instance>/
```

Common routing patterns are:

| Pattern | Purpose |
| --- | --- |
| `CONTROL/<command>` | runtime lifecycle/control |
| `SET/<client_id>` | runtime parameter mutation |
| `<client>/<ns>/REQUEST` | correlated command or plugin request |
| `<client>/<ns>/RESPONSE` | eventual correlated result |
| `<client>/<ns>/INPUT` | latest-value OCCID input stream |
| `<client>/<ns>/STATE/<route>` | rate-limited OCCID state stream |
| `DIAG/<client>/<event>` | diagnostics and lifecycle |

Request IDs, response correlation, timeouts, state-rate scheduling, and MQTT envelopes are IPC mechanics. They do not form a second semantic protocol.

For example, state remains readable with a normal MQTT client:

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'mpfc/#'
```

## Vehicle and backend selection

Vehicle configuration uses OCCID enums:

```yaml
vehicle:
  autopilot: AutopilotType.PX4
  airframe: AirframeType.COPTER
  telem_type: TelemetryType.MAVLINK
```

`main.py` resolves plugin templates, selects exactly one compatible endpoint adapter, injects it into `uav_controller`, and binds the Program to the controller.

For an INAV/MSP vehicle:

```yaml
vehicle:
  autopilot: AutopilotType.INAV
  airframe: AirframeType.COPTER
  telem_type: TelemetryType.MSP
```

## Quick start

Install Mosquitto and create an isolated Python environment:

```bash
sudo apt install mosquitto mosquitto-clients python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

If OCCID is not a sibling checkout:

```bash
export OCCID_PATH=/absolute/path/to/occid
```

Run the minimal runtime example:

```bash
MAIN_CONFIG=flight_cores/test_core/config.yaml python main.py
```

### PX4 execution host

When PX4 is already running and sending its companion MAVLink stream to UDP port `14540`:

```bash
MAIN_CONFIG=flight_cores/execution_host/config.yaml python main.py
```

This generic execution host does not implicitly arm or take off a real vehicle. Preparation policy is explicit configuration, not a side effect of receiving arbitrary work.

For local PX4 SITL with a sibling `PX4-Autopilot` checkout:

```bash
export PX4_AUTOPILOT_PATH=/absolute/path/to/PX4-Autopilot
MAIN_CONFIG=flight_cores/execution_host/config_px4_sitl.yaml python main.py
```

The SITL configuration may prepare its disposable simulated vehicle when required. The execution ingress validates the OCCID Execution, Assignment, Plan, Task, and referenced Location relationships before semantic acceptance, delegates resulting UAV commands through `uav_controller`, and determines movement completion from observed vehicle state rather than command acknowledgement alone.

`ExecutionStatusReport` is emitted as execution progresses and reaches a terminal state. MPFC does not expose the removed status-request model or a compatibility query path; higher-level reconciliation belongs to the current OCCID event/state contract.

## Other examples

PX4 SITL takeoff/land example:

```bash
MAIN_CONFIG=flight_cores/test_takeoff_land/config_px4.yaml python main.py
```

ArduPilot/MAVSDK takeoff/land example:

```bash
MAIN_CONFIG=flight_cores/test_takeoff_land/config.yaml python main.py
```

INAV/MSP example:

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

## Raspberry Pi companion appliance

`deploy/rpi/` defines the reference Raspberry Pi Zero 2 W class MPFC companion computer.

It produces one Raspberry Pi OS Lite 64-bit image that serves two purposes:

1. burn it directly to an SD card for a physical Pi Zero 2 W;
2. boot the exact same raw image under QEMU as a disposable, known-good virtual companion computer.

The image contains:

```text
/opt/mpfc
/opt/occid
/opt/hivelink
/opt/mpfc/.venv
Mosquitto
OpenSSH
systemd MPFC services
runtime and diagnostic tools
```

Build it with:

```bash
sudo apt install \
  git curl rsync xz-utils parted e2fsprogs \
  qemu-system-arm qemu-user-static binfmt-support \
  dosfstools mtools

sudo ./deploy/rpi/build-image
```

The main artifact is:

```text
deploy/rpi/dist/mpfc-rpi-zero2w.img
```

The builder also writes a SHA256 file, build manifest, and QEMU kernel/DTB sidecars. The manifest records the source revisions embedded in the image.

### Physical Pi

The physical appliance uses a MAVSDK serial URL. The default image endpoint is:

```text
serial:///dev/serial0:921600
```

The deployment procedure should explicitly assign the FC tty selected for that aircraft:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyAMA0 921600
```

A USB/UART adapter works the same way:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyUSB0 460800
```

The selected endpoint is stored in `/etc/mpfc/runtime.env` and used by `mpfc.service` on boot.

### Virtual Pi

Start the same image as a disposable virtual companion:

```bash
./deploy/rpi/pi-vm start
```

The VM uses QEMU's Pi 3A+ model with four Cortex-A53 cores and 512 MiB RAM, close to the useful compute and memory budget of a Pi Zero 2 W. Guest disk writes use QEMU snapshot mode so the base image remains known-good.

The VM overrides only the FC connection:

```text
udp://:14540
```

Useful commands:

```bash
./deploy/rpi/pi-vm ssh
./deploy/rpi/pi-vm logs
./deploy/rpi/pi-vm deploy
./deploy/rpi/pi-vm stop
```

`deploy` rsyncs the current MPFC checkout and sibling OCCID/HiveLink checkouts into the running VM and restarts MPFC, so normal Python development does not require rebuilding the SD image.

See [`deploy/rpi/README.md`](deploy/rpi/README.md) for appliance build, burn, VM, and troubleshooting details.

## Lifecycle and supervision

`main.py` starts the Program and plugins as separate processes and monitors their diagnostics:

```text
DIAG/<id>/STARTING
DIAG/<id>/ONLINE
DIAG/<id>/STOPPED
```

A child crash or `DIAG/.../ERROR` triggers coordinated shutdown through `CONTROL/SHUTDOWN`.

Shared runtime machinery lives under `lib/`:

- `common.py`: configuration, topics, MQTT routing, waits, and request correlation
- `core_base.py`: current Program base class
- `plugin_base.py`: plugin lifecycle and response helpers
- `state_scheduler.py`: rate-limited state publishing
- `occid_bus.py`: OCCID packing, unpacking, commands, and inputs
- `occid_topics.py`: local routing keys
- `uav_client.py`: Program-facing UAV convenience API
- `uav_semantics.py`: narrow MPFC adapter-routing names used to map generic OCCID Commands to endpoint mechanics
- `geo_utils.py` and `reference_frames.py`: geometry and frame helpers

On the Raspberry Pi appliance, MPFC runs as `mpfc.service` and can be watched with:

```bash
journalctl -fu mpfc.service
```

## Design rules

- Put operational meaning in OCCID, not MPFC-local schema files.
- Programs express intent and policy.
- Reusable capability logic belongs in plugins or small client facades.
- `uav_controller` accepts only the current concrete OCCID Command families; generic `Command` is not an authorization boundary.
- Every immediate Command carries a concrete `target_ref`; `uav_controller` rejects commands addressed to another asset.
- Higher-level Task/Plan/Assignment/Execution lifecycle belongs at the execution boundary, not in the endpoint adapter.
- Endpoint-specific protocol quirks and native lifecycle belong in the endpoint adapter.
- Common semantics do not require common mechanics. Do not force request/reply, streaming, acknowledgements, or native control lifecycle to look identical across protocols.
- MQTT topics route messages; they do not define semantic types.
- Keep frame, unit, datum, and native-identity conversions explicit at protocol boundaries.
- Do not create synchronous RPC semantics for high-rate direct-control samples.
- Preserve useful endpoint information rather than dropping it because it is inconvenient to represent.
- Add OCCID model coverage when a real Program or adapter demonstrates the requirement.

## Status

MPFC is pre-1.0 and evolving quickly. The current architecture is OCCID-native and is replacing the historical MPFC-specific protocol/schema layer rather than maintaining two semantic systems in parallel.

The Raspberry Pi appliance is the reference companion-computer deployment target. PX4/MAVSDK and INAV/MSP remain separate endpoint implementations behind the same Program-facing UAV boundary.
