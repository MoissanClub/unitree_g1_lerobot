"""Independent XRoboToolkit Quest video process and evidence lifecycle."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import IO, Protocol


class VideoProcess(Protocol):
    def poll(self) -> int | None: ...
    def terminate(self) -> None: ...
    def kill(self) -> None: ...
    def wait(self, timeout: float | None = None) -> int: ...


@dataclass(frozen=True, slots=True)
class QuestVideoSidecarConfig:
    sender_script: Path
    evidence_root: Path
    listen_host: str = "0.0.0.0"
    listen_port: int = 13579
    display: str = ":1"
    capture_size: str = "1920x1080"
    startup_timeout_s: float = 5.0
    python_executable: str = sys.executable

    def __post_init__(self) -> None:
        object.__setattr__(self, "sender_script", Path(self.sender_script))
        object.__setattr__(self, "evidence_root", Path(self.evidence_root))
        if not self.listen_host or not 1 <= self.listen_port <= 65535:
            raise ValueError("listen host and port must be valid")
        if self.startup_timeout_s <= 0:
            raise ValueError("startup_timeout_s must be positive")


@dataclass(frozen=True, slots=True)
class QuestVideoSidecarStatus:
    state: str
    detail: str
    evidence_directory: str | None
    exit_code: int | None = None


class QuestVideoSidecar:
    """Start/stop video independently; startup failure is reported, not raised."""

    def __init__(self, config: QuestVideoSidecarConfig, *, popen=subprocess.Popen) -> None:
        self.config = config
        self._popen = popen
        self._process: VideoProcess | None = None
        self._log: IO[bytes] | None = None
        self._directory: Path | None = None
        self._started_at: str | None = None
        self._status = QuestVideoSidecarStatus("disabled", "not started", None)

    @property
    def status(self) -> QuestVideoSidecarStatus:
        if self._process is not None:
            code = self._process.poll()
            if code is not None and self._status.state == "ready":
                self._status = QuestVideoSidecarStatus(
                    "failed", "video sender exited", str(self._directory), code
                )
        return self._status

    def _ready(self) -> bool:
        host = "127.0.0.1" if self.config.listen_host == "0.0.0.0" else self.config.listen_host
        try:
            with socket.create_connection((host, self.config.listen_port), timeout=0.1):
                return True
        except OSError:
            return False

    def start(self) -> QuestVideoSidecarStatus:
        if self._process is not None:
            raise RuntimeError("video sidecar is already started")
        stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
        self._directory = self.config.evidence_root / f"quest-video-{stamp}"
        self._directory.mkdir(parents=True, exist_ok=False)
        self._log = (self._directory / "sender.log").open("xb")
        self._started_at = datetime.now(UTC).isoformat()
        command = [
            self.config.python_executable,
            str(self.config.sender_script),
            "--listen",
            f"{self.config.listen_host}:{self.config.listen_port}",
            "--display",
            self.config.display,
            "--capture-size",
            self.config.capture_size,
        ]
        try:
            self._process = self._popen(command, stdout=self._log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + self.config.startup_timeout_s
            while time.monotonic() < deadline:
                code = self._process.poll()
                if code is not None:
                    self._status = QuestVideoSidecarStatus(
                        "failed", "video sender exited during startup", str(self._directory), code
                    )
                    break
                if self._ready():
                    self._status = QuestVideoSidecarStatus(
                        "ready", "waiting for Quest OPEN_CAMERA", str(self._directory)
                    )
                    break
                time.sleep(0.05)
            else:
                self._status = QuestVideoSidecarStatus(
                    "failed", "video sender readiness timeout", str(self._directory)
                )
        except (OSError, subprocess.SubprocessError) as exc:
            self._status = QuestVideoSidecarStatus(
                "failed", f"video sender start failed: {exc}", str(self._directory)
            )
        self._write_metadata()
        return self._status

    def _write_metadata(self) -> None:
        if self._directory is None:
            return
        payload = {
            "started_at_utc": self._started_at,
            "updated_at_utc": datetime.now(UTC).isoformat(),
            "config": {key: str(value) for key, value in asdict(self.config).items()},
            "status": asdict(self.status),
        }
        (self._directory / "metadata.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n"
        )

    def stop(self) -> QuestVideoSidecarStatus:
        process, self._process = self._process, None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                code = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                code = process.wait(timeout=2)
            if self._status.state == "ready":
                self._status = QuestVideoSidecarStatus(
                    "stopped", "video sender stopped", str(self._directory), code
                )
        elif process is not None and self._status.exit_code is None:
            code = process.poll()
            self._status = QuestVideoSidecarStatus(
                "failed", "video sender had already exited", str(self._directory), code
            )
        if self._log is not None:
            self._log.close()
            self._log = None
        self._write_metadata()
        return self._status
