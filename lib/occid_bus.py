"""OCCID transport helpers for the MPFC MQTT runtime.

OCCID remains the semantic model. MQTT envelopes only carry OCCID's own
versioned MsgPack encoding as base64 so the JSON MQTT wrapper does not invent a
second representation of OCCID.
"""

from __future__ import annotations

import base64
import os
import sys
from pathlib import Path
from typing import Any

import msgpack


def _load_occid_schema():
    try:
        import schema as candidate
        if hasattr(candidate, "OCCIDModel") and hasattr(candidate, "OCCID_MODEL_BY_ID"):
            return candidate
    except ImportError:
        pass

    candidates: list[Path] = []
    configured = os.environ.get("OCCID_PATH")
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(Path(__file__).resolve().parents[2] / "occid")

    for path in candidates:
        if not path.exists():
            continue
        path_text = str(path.resolve())
        if path_text not in sys.path:
            sys.path.insert(0, path_text)
        try:
            import schema as candidate
        except ImportError:
            continue
        if hasattr(candidate, "OCCIDModel") and hasattr(candidate, "OCCID_MODEL_BY_ID"):
            return candidate

    searched = ", ".join(str(path) for path in candidates)
    raise RuntimeError(
        "OCCID schema package not found. Set OCCID_PATH to the OCCID repository root "
        f"or place it beside MPFC. searched={searched}"
    )


occid = _load_occid_schema()
OCCID_PAYLOAD_KEY = "occid_b64"


def is_occid_model(value: Any) -> bool:
    return isinstance(value, occid.OCCIDModel)


def pack_occid(model: Any) -> dict[str, str]:
    if not is_occid_model(model):
        raise TypeError(f"expected OCCID model, got {type(model).__name__}")
    encoded = model.encode()
    return {OCCID_PAYLOAD_KEY: base64.b64encode(encoded).decode("ascii")}


def unpack_occid(payload: Any) -> Any:
    if type(payload) is not dict or set(payload) != {OCCID_PAYLOAD_KEY}:
        raise ValueError(f"invalid OCCID bus payload type={type(payload).__name__}")
    encoded = base64.b64decode(payload[OCCID_PAYLOAD_KEY], validate=True)
    envelope = msgpack.unpackb(encoded, raw=False)
    model_id = int(envelope["model_id"])
    model_type = occid.OCCID_MODEL_BY_ID.get(model_id)
    if model_type is None:
        raise ValueError(f"unknown OCCID model id {model_id}")
    return model_type.decode(encoded)


def get_occid_state(state: dict[str, Any], key: str, expected_type: type | tuple[type, ...] | None = None) -> Any:
    payload = state.get(key)
    if payload is None:
        return None
    model = unpack_occid(payload)
    if expected_type is not None and not isinstance(model, expected_type):
        raise TypeError(
            f"unexpected OCCID state key={key} expected={expected_type} actual={type(model).__name__}"
        )
    return model


def send_occid_command(router: Any, request_topic: str, command: Any) -> str:
    if not isinstance(command, occid.Command):
        raise TypeError(f"expected OCCID Command, got {type(command).__name__}")
    router.request_counter += 1
    request_id = f"req-{router.request_counter}"
    payload = {
        "request_id": request_id,
        "command": pack_occid(command),
    }
    from lib.common import build_envelope

    router.publish(request_topic, build_envelope(router.client_id, request_topic, payload))
    return request_id


def decode_occid_command(request: dict[str, Any]) -> tuple[str, Any]:
    request_id = str(request["request_id"])
    command = unpack_occid(request["command"])
    if not isinstance(command, occid.Command):
        raise TypeError(f"request payload is not OCCID Command actual={type(command).__name__}")
    return request_id, command
