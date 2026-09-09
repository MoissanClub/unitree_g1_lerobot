"""Bridge-owned startup diagnostic request handling."""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from unitree_g1_lerobot.robots.control import publish_ready_for


class StartupDiagnosticRequests:
    def __init__(
        self,
        request_file: Path,
        ack_file: Path,
        raise_action: dict[str, float],
        lower_action: dict[str, float],
        hz: float,
    ) -> None:
        self.request_file = request_file
        self.ack_file = ack_file
        self.raise_action = raise_action
        self.lower_action = lower_action
        self.hz = hz
        self.active_id: str | None = None

    def read_request(self) -> dict | None:
        try:
            return json.loads(self.request_file.read_text())
        except FileNotFoundError:
            self.active_id = None
            return None
        except (OSError, json.JSONDecodeError) as exc:
            print(f"Ignoring invalid startup diagnostic request: {exc}", file=sys.stderr, flush=True)
            return None

    def write_ack(self, request_id: str) -> None:
        try:
            self.ack_file.write_text(json.dumps({"id": request_id, "status": "active"}) + "\n")
        except OSError as exc:
            print(f"Could not write startup diagnostic ack: {exc}", file=sys.stderr, flush=True)

    def step(self, robot) -> bool:
        request = self.read_request()
        if not request or request.get("mode") != "lower_hold":
            return False

        request_id = str(request.get("id", ""))
        if request_id != self.active_id:
            self.active_id = request_id
            print("Startup diagnostic request: lowering both arms until user confirmation.", flush=True)
            publish_ready_for(robot, self.raise_action, 1.0, self.hz)
            self.write_ack(request_id)

        robot.send_action(self.lower_action)
        return True

    def publish_or_ready_for(self, robot, ready_action: dict[str, float], duration_s: float) -> None:
        if robot is None:
            time.sleep(max(0.0, duration_s))
            return
        period_s = 1.0 / self.hz
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            t0 = time.perf_counter()
            if not self.step(robot):
                robot.send_action(ready_action)
            time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
