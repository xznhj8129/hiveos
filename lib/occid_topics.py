"""Local MQTT routing keys for OCCID state streams.

These names identify streams for subscription/rate control only. The OCCID model
inside each payload defines its semantics.
"""

FLIGHT_CONTROL = "flight_control"
LOCATION = "location"
ATTITUDE = "attitude"
ANGULAR_VELOCITY = "angular_velocity"
GNSS = "gnss"
POWER = "power"
IMU = "imu"
FIRMWARE = "firmware"
CONTROL_OVERRIDE = "control_override"
CONTROL_OUTPUT = "control_output"
