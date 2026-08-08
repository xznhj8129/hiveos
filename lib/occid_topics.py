"""Local MQTT routing keys for OCCID state streams.

These names identify streams for subscription/rate control only. The OCCID model
inside each payload defines its semantics.
"""

FLIGHT_CONTROL = "flight_control"
LOCATION = "location"
ATTITUDE = "attitude"
ANGULAR_VELOCITY = "angular_velocity"
GNSS = "gnss"
AUTOPILOT_MISSION = "autopilot_mission"
POWER = "power"
IMU = "imu"
FIRMWARE = "firmware"
SENSOR_CONFIG = "sensor_config"
CONTROL_OVERRIDE = "control_override"
CONTROL_OUTPUT = "control_output"
RC_TELEMETRY = "rc_telemetry"
REMOTE_CONTROL = "remote_control"
RUNTIME_LOAD = "runtime_load"
TRACKER = "tracker"
ENTITY_STATE = "entity_state"
HUMAN_TEXT = "human_text"
COT_RAW = "cot_raw"
