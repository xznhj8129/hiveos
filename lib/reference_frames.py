#!/usr/bin/env python3
"""
Reference-frame conversion helpers.

MPFC keeps these compatibility names for existing callers, but the canonical
representation conversion lives in the OCCID SDK interop layer.
"""

from typing import Any, Dict

from interop.common import fru_to_frd_vector as _occid_fru_to_frd_vector

FRAME_FRD = "FRD"


def up_to_ned_down(value: float | None) -> float | None:
    if value is None:
        return None
    return -value


def ned_down_to_up(value: float | None) -> float | None:
    if value is None:
        return None
    return -value


def fru_to_frd_vector(x: float, y: float, z_up: float) -> tuple[float, float, float]:
    return _occid_fru_to_frd_vector(x, y, z_up)


def rc_dict_to_aetr(values: Dict[str, Any]) -> list[Any]:
    return [
        values.get("roll"),
        values.get("pitch"),
        values.get("throttle"),
        values.get("yaw"),
    ]


def rc_rpyt_to_aetr(roll: float, pitch: float, yaw: float, throttle: float) -> list[float]:
    return [roll, pitch, throttle, yaw]
