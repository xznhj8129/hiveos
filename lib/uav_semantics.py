"""MPFC-local mapping names for generic OCCID UAV Commands.

These are adapter routing conventions, not a second semantic model. OCCID owns
the Command families; MPFC maps their typed operations and operands to endpoint
mechanics.
"""
from __future__ import annotations

from typing import Any


PROPERTY_ARMED = "armed"
PROPERTY_STANDARD_FLIGHT_MODE = "standard_flight_mode"
PROPERTY_NATIVE_FLIGHT_MODE_NAME = "native_flight_mode_name"
PROPERTY_NATIVE_FLIGHT_MODE_CODE = "native_flight_mode_code"

PARAM_TAKEOFF_ALTITUDE_M = "takeoff_altitude_m"

PROCESS_TAKEOFF = "takeoff"
PROCESS_LAND = "land"
PROCESS_RETURN_TO_LAUNCH = "return_to_launch"
PROCESS_DIRECT_CONTROL = "direct_control"
PROCESS_DIRECT_CONTROL_ATTITUDE = "direct_control.attitude_thrust"
PROCESS_DIRECT_CONTROL_MANUAL = "direct_control.manual_axis"

DIRECT_CONTROL_ATTITUDE = "ATTITUDE_THRUST"
DIRECT_CONTROL_MANUAL = "MANUAL_AXIS"


def metadata_scalar(value: Any) -> Any:
    """Return the one populated scalar from an OCCID MetadataValue."""
    if value is None:
        return None
    populated = [
        getattr(value, name)
        for name in ("str", "int", "float", "bool")
        if getattr(value, name) is not None
    ]
    if len(populated) != 1:
        raise ValueError("MetadataValue must contain exactly one scalar value")
    return populated[0]
