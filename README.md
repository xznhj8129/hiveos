# MPFC
## Multi-Protocol Flight Computer
*Monty Python's Flying Clankers*

MPFC is a lightweight resident companion-computer runtime for autonomous and remotely controlled robotic systems.

A user-written **Program** expresses behavior. Reusable **plugins** provide capabilities such as UAV control, MAVLink/MAVSDK, MSP, CoT/ATAK, tracking, vision, and datalinks. **OCCID** is the shared semantic model used between components. MQTT is private node-local IPC and routing.

MPFC deliberately avoids becoming a large framework. Runtime behavior is ordinary Python processes, explicit configuration, readable Programs, and small adapters around real external systems.

The source still contains historical names such as `flight_cores/`, `CoreBase`, and `run_core()` for Programs. Those names are cleanup debt, not a second runtime model.

## Architecture

```text
external OCCID work
       |
   HiveLink
       |
execution_ingress
       |
local capability policy
       |
 uav_controller
       |
 +-----+------------------+
 |                        |
MAVSDK/MAVLink           MSP
 |                        |
PX4 / ArduPilot      INAV / Betaflight
```

Important boundaries:

- **Program** - main behavior being run.
- **Plugin** - reusable capability logic or external-system interface.
- **`uav_controller`** - stable Program-facing UAV capability and vehicle readiness/command policy.
- **Endpoint adapters** - translate OCCID to/from native mechanisms such as MAVSDK/MAVLink, MSP, or CoT.
- **`execution_ingress`** - accepts higher-level OCCID Task/Plan/Assignment/Execution work and delegates concrete local behavior through normal capabilities.
- **MQTT** - local IPC. Topics are routing, not semantic types.
- **OCCID** - shared operational meaning.
- **HiveLink** - OCCID delivery between independently deployed nodes.

A component that only translates OCCID into another MPFC-specific semantic vocabulary is probably in the wrong place.

## OCCID dependency

OCCID is a normal installed Python dependency. For a source checkout:

```bash
python -m pip install -e ../occid
```

The integrated Sigmac3 runtime installs `/opt/occid` into the MPFC guest virtualenv and verifies MPFC's generated `OCCID_CONTRACT` before startup.

Do **not** restore `OCCID_PATH`, sibling-checkout import discovery, or OCCID `PYTHONPATH` mutation.

HiveLink is installed separately when the deployment uses the external OCCID delivery boundary.

## Program style

Programs should read like intent rather than protocol plumbing. The convenience API in `lib/uav_client.py` constructs and consumes OCCID models:

```python
uav.set_takeoff_altitude(10.0)
uav.arm()
uav.takeoff()
uav.go_to(lat, lon, 20.0)
uav.return_to_launch()
uav.land()
```

Immediate operations use current OCCID Command families such as `StateChangeCommand`, `ProcessControlCommand`, `ConfigurationCommand`, and `MotionCommand`.

High-rate direct control is separate from command RPC. A Program acquires the endpoint-local control process, publishes latest-value OCCID `Input` samples, then releases the process. Session mechanics such as MAVSDK offboard/manual control or MSP RC override remain endpoint/runtime behavior rather than new OCCID command hierarchies.

## Higher-level execution

Task/Plan/Assignment/Execution records do not go through `uav_controller` merely because they eventually produce vehicle commands. `execution_ingress` owns that higher-level lifecycle and semantic admission.

The proven path accepts `TaskManeuver` with `ManeuverIntent.MOVE`, resolves its referenced Location, delegates movement through `uav_controller`, and determines completion from observed state rather than command ACK alone.

```text
Sigma
  -> Plan / TaskManeuver / Assignment / Execution
  -> HiveLink
  -> MPFC execution_ingress
  -> uav_controller
  -> MAVSDK
  -> PX4
  -> ExecutionAcceptance / ExecutionStatusReport / observed EntityState
```

Unsupported Task families/intents are rejected rather than guessed into endpoint behavior.

## Local OCCID IPC

MPFC keeps local MQTT traffic human-readable for diagnosis. OCCID model identity may be tagged in the readable JSON representation, but a schema-version compatibility field is **not** required on every local or transient payload.

Compact OCCID transport encoding is the model's permanent `model_id` plus named fields. Structural compatibility is handled by OCCID consumer contracts at development/deployment time.

Representative state routes include location, attitude, angular velocity, GNSS, flight control, power, sensor configuration, remote-control state, runtime load, tracker state, and aggregate entity state. Route strings are not types; the OCCID model carried in the data defines meaning.

## Reference-frame contract

Unless a record explicitly states otherwise:

- body frame: FRD;
- inertial/world frame: NED;
- angular values: radians;
- angular velocity: radians/second;
- altitude: meters with explicit datum where needed;
- normalized control axes: signed `-1..+1` semantic position;
- endpoint adapters own PWM/native-unit conversion.

CoT HAE is WGS84 ellipsoid height, not mean-sea-level altitude. External protocol IDs are not automatically OCCID domain identities.

## Development runtime

Normal Conqueror Frog integration development uses the Sigmac3-managed Ubuntu 24.04 x86 KVM guest with host PX4 SIH. The resident runtime reports READY only after MPFC, MAVSDK, and live `LocationState` are online.

The dedicated configuration is:

```text
flight_cores/execution_host/config_x86_dev.yaml
```

Lower-level standalone execution-host startup remains available when required:

```bash
MAIN_CONFIG=flight_cores/execution_host/config.yaml python main.py
```

The execution host does not implicitly arm or take off a physical vehicle merely because work arrives. Preparation policy remains explicit.

## Raspberry Pi companion target

Raspberry Pi Zero 2 W remains the intended small physical companion target. `deploy/rpi/` contains the image/qualification machinery.

The Pi image and Pi-QEMU path are **qualification tools**, not the normal integration feedback loop. Normal Block 1/2 debugging should use the x86 KVM runtime managed by Sigmac3.

Physical flight-controller endpoints are explicit serial URLs such as:

```text
serial:///dev/serial0:921600
```

See `deploy/rpi/README.md` for target-image details.

## Lifecycle and supervision

`main.py` starts the Program and plugins as separate processes and monitors diagnostics. Unexpected child failure triggers coordinated shutdown rather than silently leaving a partial runtime alive.

Shared runtime machinery under `lib/` includes configuration/MQTT routing, Program/plugin base classes, state scheduling, OCCID packing/routing, UAV client semantics, geometry, and reference-frame helpers.

## Design rules

- Put operational meaning in OCCID, not MPFC-local schema files.
- Programs express intent and policy.
- Reusable capability logic belongs in plugins/small facades.
- Higher-level Task/Plan/Assignment/Execution lifecycle belongs at the execution boundary, not endpoint adapters.
- Endpoint-specific connection/session quirks remain in endpoint adapters/runtime code.
- Common semantics do not require identical mechanics across protocols.
- MQTT topics route messages; they do not define types.
- Keep frame, unit, datum, identity, sentinel, and range conversion explicit.
- Do not turn high-rate input streams into fake synchronous RPC.
- Add semantic coverage when a real Program/adapter demonstrates the requirement.
- Remove obsolete paths rather than preserving compatibility shims.

## Status

The normal Sigma -> HiveLink -> resident MPFC -> MAVSDK/PX4 execution path has passed target-host end-to-end acceptance, including visible progress and terminal success in Sigma. Current exact integrated heads and evidence are maintained in the Conqueror Frog `STATE.md` authority.

MPFC remains pre-1.0 and evolving. Later work should broaden/harden Programs, cancellation/recovery, endpoint coverage, and physical target qualification without reopening the accepted architecture.
