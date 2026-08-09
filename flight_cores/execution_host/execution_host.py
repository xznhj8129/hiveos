#!/usr/bin/env python3
"""Passive Program host for externally initiated MPFC executions.

The execution_ingress plugin owns high-level request handling. This Core keeps
the normal MPFC supervisor lifecycle intact without launching a competing
startup mission. Existing autonomous Programs remain unchanged and can later be
selected by richer ingress handlers.
"""
from __future__ import annotations

import time
from typing import Any, Dict

from lib.core_base import CoreBase


class ExecutionHost(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.poll_interval_s = float(cfg.get("poll_interval_s", 0.25))

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                deadline = time.monotonic() + self.poll_interval_s
                self.recv_until(deadline)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    ExecutionHost(cfg, bus_config).run()
