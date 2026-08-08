"""Minimal Program/plugin OCCID request-response smoke test."""

from __future__ import annotations

import time
from typing import Any, Dict

from lib.common import build_request_topic, build_response_topic
from lib.core_base import CoreBase
from lib.occid_bus import occid, send_occid_request, unpack_occid


class HelloCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.count = int(cfg["count"])
        self.send_interval = float(cfg["send_interval"])
        self.response_timeout_s = float(cfg["response_timeout_s"])
        self.targets = list(cfg["targets"])
        self.init_bus(float(cfg["poll_interval_s"]))
        self.target_topics: list[tuple[str, str, str]] = []
        for target in self.targets:
            request_topic = build_request_topic(target["id"], target["topic_ns"])
            response_topic = build_response_topic(target["id"], target["topic_ns"])
            self.bus.add_response_topic(response_topic)
            self.target_topics.append((target["id"], request_topic, response_topic))

    def run(self) -> None:
        self.send_online()
        try:
            for iteration in range(self.count):
                for target_id, request_topic, _response_topic in self.target_topics:
                    request = occid.ProtocolPayload(
                        format=occid.ProtocolPayloadFormat.TEXT,
                        content_type="text/plain",
                        text=f"hello {target_id} #{iteration}",
                    )
                    request_id = send_occid_request(self.bus, request_topic, request)
                    response = self._wait_response(request_id, self.response_timeout_s)
                    if not response.get("ok"):
                        raise RuntimeError(f"hello request failed target={target_id} response={response}")
                    reply = unpack_occid(response["data"]["reply"])
                    if not isinstance(reply, occid.ProtocolPayload):
                        raise RuntimeError(f"invalid hello reply type {type(reply).__name__}")
                    print(
                        f"[CORE] {self.client_id} target={target_id} request_id={request_id} reply={reply.text!r}",
                        flush=True,
                    )
                if iteration + 1 < self.count:
                    time.sleep(self.send_interval)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    HelloCore(cfg, bus_config).run()
