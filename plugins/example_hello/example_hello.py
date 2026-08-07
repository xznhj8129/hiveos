#!/usr/bin/env python3
"""Minimal OCCID plugin request/response smoke-test endpoint."""

from __future__ import annotations

import time
import traceback
from typing import Any, Dict

from lib.common import build_request_topic, build_response_topic
from lib.occid_bus import decode_occid_request, occid, pack_occid
from lib.plugin_base import PluginBase


class HelloPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.send_interval = float(cfg["send_interval"])
        self.topic_ns = str(cfg["topic_ns"])
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(self.poll_interval_s)

    def _handle_request(self, payload: Dict[str, Any]) -> None:
        request_id, model = decode_occid_request(payload)
        if not isinstance(model, occid.ProtocolPayload):
            self.enqueue_response(
                request_id,
                type(model).__name__,
                False,
                {"error": f"expected ProtocolPayload actual={type(model).__name__}"},
            )
            return
        if model.format != occid.ProtocolPayloadFormat.TEXT or model.text is None:
            self.enqueue_response(
                request_id,
                type(model).__name__,
                False,
                {"error": "hello smoke test requires TEXT ProtocolPayload"},
            )
            return
        reply = occid.ProtocolPayload(
            format=occid.ProtocolPayloadFormat.TEXT,
            content_type="text/plain",
            text=f"{model.text} -> pong from {self.client_id}",
        )
        self.enqueue_response(
            request_id,
            type(model).__name__,
            True,
            {"reply": pack_occid(reply)},
        )
        print(
            f"[{self.client_id}] request_id={request_id} message={model.text!r} reply={reply.text!r}",
            flush=True,
        )

    def run(self) -> None:
        self.send_online()
        deadline = time.monotonic() + self.send_interval
        try:
            while True:
                topic, payload = self.recv_until(deadline)
                self.flush_queue(self.response_queue, self.response_topic)
                if topic == self.request_topic:
                    self._handle_request(payload["data"])
                    self.flush_queue(self.response_queue, self.response_topic)
                deadline = time.monotonic() + self.send_interval
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    HelloPlugin(cfg, bus_config).run()
