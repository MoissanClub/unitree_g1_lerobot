"""Quest OPEN_CAMERA/H.264 protocol and independent sidecar tests."""

from __future__ import annotations

import io
import socket
import struct
from pathlib import Path

from tools.quest_video_sender import access_units, parse_camera_request, receive_command
from unitree_g1_lerobot.xr import QuestVideoSidecar, QuestVideoSidecarConfig


class NeverStop:
    @staticmethod
    def is_set():
        return False


def camera_payload():
    values = struct.pack("<7i", 2560, 720, 60, 4_000_000, 0, 2, 12345)
    ip = b"10.9.0.127"
    return b"\xca\xfe\x01" + values + b"\x03ZED" + bytes([len(ip)]) + ip


def test_open_camera_request_and_framed_control_command():
    request = parse_camera_request(camera_payload())
    assert (request.width, request.height, request.port) == (2560, 720, 12345)
    command = b"OPEN_CAMERA"
    payload = camera_payload()
    body = struct.pack("<i", len(command)) + command + struct.pack("<i", len(payload)) + payload
    sender, receiver = socket.socketpair()
    try:
        sender.sendall(struct.pack(">I", len(body)) + body)
        assert receive_command(receiver) == ("OPEN_CAMERA", payload)
    finally:
        sender.close()
        receiver.close()


def test_h264_access_units_are_split_at_aud_and_length_frameable():
    aud = b"\x00\x00\x00\x01\x09\x10"
    units = list(access_units(io.BytesIO(aud + b"first" + aud + b"second"), NeverStop()))
    assert units == [aud + b"first", aud + b"second"]
    framed = b"".join(struct.pack(">I", len(unit)) + unit for unit in units)
    assert struct.unpack_from(">I", framed)[0] == len(units[0])


class FakeProcess:
    def __init__(self, code=None):
        self.code = code
        self.terminated = False

    def poll(self):
        return self.code

    def terminate(self):
        self.terminated = True
        self.code = 0

    def kill(self):
        self.code = -9

    def wait(self, timeout=None):
        return self.code


def config(tmp_path):
    return QuestVideoSidecarConfig(
        sender_script=Path("tools/quest_video_sender.py"),
        evidence_root=tmp_path,
        startup_timeout_s=0.01,
    )


def test_sidecar_records_ready_and_ordered_shutdown(monkeypatch, tmp_path):
    process = FakeProcess()
    sidecar = QuestVideoSidecar(config(tmp_path), popen=lambda *args, **kwargs: process)
    monkeypatch.setattr(sidecar, "_ready", lambda: True)
    started = sidecar.start()
    assert started.state == "ready"
    stopped = sidecar.stop()
    assert stopped.state == "stopped"
    assert process.terminated
    directory = Path(stopped.evidence_directory)
    assert (directory / "sender.log").is_file()
    assert '"state": "stopped"' in (directory / "metadata.json").read_text()


def test_sender_failure_is_reported_without_raising(tmp_path):
    process = FakeProcess(code=7)
    sidecar = QuestVideoSidecar(config(tmp_path), popen=lambda *args, **kwargs: process)
    status = sidecar.start()
    assert status.state == "failed"
    assert status.exit_code == 7
    sidecar.stop()


def test_real_sender_has_bounded_ready_and_shutdown_lifecycle(tmp_path):
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    sidecar = QuestVideoSidecar(
        QuestVideoSidecarConfig(
            sender_script=Path("tools/quest_video_sender.py").resolve(),
            evidence_root=tmp_path,
            listen_host="127.0.0.1",
            listen_port=port,
            startup_timeout_s=2.0,
        )
    )
    assert sidecar.start().state == "ready"
    assert sidecar.stop().state == "stopped"
