#!/usr/bin/env python3
"""Stream an X11 desktop view to XRoboToolkit-Quest's video decoder.

The Quest app connects to the control port and sends an OPEN_CAMERA request.
This sender then connects back to the headset's video port and sends H.264
access units, each prefixed by a four-byte big-endian length.  The captured
desktop is scaled to one eye and duplicated side-by-side for comfortable
monoscopic viewing through the app's stereo presentation path.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import signal
import socket
import struct
import subprocess
import threading
import time
from collections.abc import Iterator


LOG = logging.getLogger("quest-video-sender")
AUD_PATTERNS = (b"\x00\x00\x00\x01\x09", b"\x00\x00\x01\x09")


@dataclasses.dataclass(frozen=True)
class CameraRequest:
    width: int
    height: int
    fps: int
    bitrate: int
    enable_hevc: int
    render_mode: int
    port: int
    camera: str
    ip: str


def _compact_string(data: bytes, offset: int) -> tuple[str, int]:
    if offset >= len(data):
        raise ValueError("missing compact-string length")
    length = data[offset]
    offset += 1
    end = offset + length
    if end > len(data):
        raise ValueError("truncated compact string")
    return data[offset:end].decode("utf-8"), end


def parse_camera_request(data: bytes) -> CameraRequest:
    if len(data) < 31 or data[:3] != b"\xca\xfe\x01":
        raise ValueError("invalid camera-request header")
    values = struct.unpack_from("<7i", data, 3)
    camera, offset = _compact_string(data, 31)
    ip, offset = _compact_string(data, offset)
    if offset != len(data):
        raise ValueError("unexpected trailing camera-request bytes")
    return CameraRequest(*values, camera=camera, ip=ip)


def receive_command(conn: socket.socket) -> tuple[str, bytes] | None:
    wrapper = _receive_exact(conn, 4)
    if wrapper is None:
        return None
    body_length = struct.unpack(">I", wrapper)[0]
    if not 8 <= body_length <= 65536:
        raise ValueError(f"invalid wrapped body length: {body_length}")
    body = _receive_exact(conn, body_length)
    if body is None:
        raise EOFError("connection closed in wrapped body")

    command_length = struct.unpack_from("<i", body, 0)[0]
    if not 0 <= command_length <= 256:
        raise ValueError(f"invalid command length: {command_length}")
    data_length_offset = 4 + command_length
    if data_length_offset + 4 > len(body):
        raise ValueError("wrapped body is truncated before data length")
    command_bytes = body[4:data_length_offset]
    data_length = struct.unpack_from("<i", body, data_length_offset)[0]
    if not 0 <= data_length <= 65536:
        raise ValueError(f"invalid data length: {data_length}")
    data_offset = data_length_offset + 4
    if data_offset + data_length != len(body):
        raise ValueError(
            f"wrapped body size mismatch: declared {data_length}, "
            f"available {len(body) - data_offset}"
        )
    data = body[data_offset:]
    return command_bytes.rstrip(b"\x00").decode("utf-8"), data


def _receive_exact(conn: socket.socket, length: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = length
    while remaining:
        chunk = conn.recv(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _aud_positions(data: bytes) -> list[int]:
    positions: list[int] = []
    index = 0
    while index < len(data):
        if data.startswith(AUD_PATTERNS[0], index):
            positions.append(index)
            index += len(AUD_PATTERNS[0])
        elif data.startswith(AUD_PATTERNS[1], index):
            positions.append(index)
            index += len(AUD_PATTERNS[1])
        else:
            index += 1
    return positions


def access_units(stream, stop: threading.Event) -> Iterator[bytes]:
    buffer = bytearray()
    while not stop.is_set():
        chunk = stream.read(65536)
        if not chunk:
            break
        buffer.extend(chunk)
        positions = _aud_positions(buffer)
        while len(positions) >= 2:
            start, end = positions[0], positions[1]
            if start:
                del buffer[:start]
                end -= start
            yield bytes(buffer[:end])
            del buffer[:end]
            positions = _aud_positions(buffer)
    if buffer and _aud_positions(buffer):
        start = _aud_positions(buffer)[0]
        yield bytes(buffer[start:])


class StreamWorker:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.process: subprocess.Popen | None = None
        self.video_socket: socket.socket | None = None

    def start(self, request: CameraRequest, peer_ip: str) -> None:
        self.stop()
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run, args=(request, peer_ip), daemon=True
        )
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if self.video_socket:
            try:
                self.video_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.video_socket.close()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=5)
        self.thread = None
        self.process = None
        self.video_socket = None

    def _run(self, request: CameraRequest, peer_ip: str) -> None:
        target_ip = request.ip or peer_ip
        if request.ip and request.ip != peer_ip:
            LOG.warning(
                "request target %s differs from control peer %s", request.ip, peer_ip
            )
        if request.enable_hevc:
            LOG.error("HEVC was requested; this experiment supports H.264 only")
            return
        if request.width < 2 or request.width % 2 or request.height < 2:
            LOG.error("unsupported dimensions: %sx%s", request.width, request.height)
            return

        try:
            self.video_socket = self._connect_video(target_ip, request.port)
            command = self._ffmpeg_command(request)
            LOG.info("starting capture: %s", " ".join(command))
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
            assert self.process.stdout is not None
            assert self.process.stderr is not None
            stderr_thread = threading.Thread(
                target=self._log_stderr, args=(self.process.stderr,), daemon=True
            )
            stderr_thread.start()

            frames = 0
            total_bytes = 0
            started = time.monotonic()
            for unit in access_units(self.process.stdout, self.stop_event):
                if not unit:
                    continue
                self.video_socket.sendall(struct.pack(">I", len(unit)) + unit)
                frames += 1
                total_bytes += len(unit)
                if frames == 1:
                    LOG.info("first H.264 access unit sent (%d bytes)", len(unit))
                elif frames % 300 == 0:
                    elapsed = max(time.monotonic() - started, 0.001)
                    LOG.info(
                        "sent %d frames, %.1f fps, %.2f Mbit/s",
                        frames,
                        frames / elapsed,
                        total_bytes * 8 / elapsed / 1_000_000,
                    )
        except (OSError, subprocess.SubprocessError) as exc:
            if not self.stop_event.is_set():
                LOG.error("stream failed: %s", exc)
        finally:
            self.stop_event.set()
            LOG.info("stream ended")

    def _connect_video(self, ip: str, port: int) -> socket.socket:
        deadline = time.monotonic() + self.args.connect_timeout
        last_error: OSError | None = None
        while time.monotonic() < deadline and not self.stop_event.is_set():
            try:
                connection = socket.create_connection((ip, port), timeout=2)
                connection.settimeout(None)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                LOG.info("connected to Quest video receiver at %s:%d", ip, port)
                return connection
            except OSError as exc:
                last_error = exc
                time.sleep(0.2)
        raise OSError(f"could not connect to {ip}:{port}: {last_error}")

    def _ffmpeg_command(self, request: CameraRequest) -> list[str]:
        fps = min(max(request.fps, 1), self.args.max_fps)
        bitrate = min(max(request.bitrate, 250_000), self.args.max_bitrate)
        eye_width = request.width // 2
        filter_graph = (
            f"scale={eye_width}:{request.height}:flags=fast_bilinear,"
            "split=2[left][right];[left][right]hstack=inputs=2"
        )
        return [
            self.args.ffmpeg,
            "-hide_banner",
            "-loglevel",
            self.args.ffmpeg_loglevel,
            "-f",
            "x11grab",
            "-draw_mouse",
            "0",
            "-video_size",
            self.args.capture_size,
            "-framerate",
            str(fps),
            "-i",
            f"{self.args.display}+{self.args.capture_x},{self.args.capture_y}",
            "-vf",
            filter_graph,
            "-an",
            "-c:v",
            "h264_nvenc",
            "-preset",
            "p1",
            "-tune",
            "ull",
            "-zerolatency",
            "1",
            "-rc-lookahead",
            "0",
            "-g",
            str(fps),
            "-bf",
            "0",
            "-b:v",
            str(bitrate),
            "-maxrate",
            str(bitrate),
            "-bufsize",
            str(max(bitrate // 2, 250_000)),
            "-profile:v",
            "main",
            "-level:v",
            "4.2",
            "-aud",
            "1",
            "-forced-idr",
            "1",
            "-f",
            "h264",
            "pipe:1",
        ]

    @staticmethod
    def _log_stderr(stream) -> None:
        for raw_line in iter(stream.readline, b""):
            line = raw_line.decode("utf-8", errors="replace").rstrip()
            if line:
                LOG.info("ffmpeg: %s", line)


def parse_address(value: str) -> tuple[str, int]:
    host, separator, port_text = value.rpartition(":")
    if not separator or not host:
        raise argparse.ArgumentTypeError("expected HOST:PORT")
    try:
        port = int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return host, port


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen", type=parse_address, default=("0.0.0.0", 13579))
    parser.add_argument("--display", default=":1")
    parser.add_argument("--capture-size", default="1920x1080")
    parser.add_argument("--capture-x", type=int, default=0)
    parser.add_argument("--capture-y", type=int, default=0)
    parser.add_argument("--max-fps", type=int, default=60)
    parser.add_argument("--max-bitrate", type=int, default=8_000_000)
    parser.add_argument("--connect-timeout", type=float, default=8.0)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--ffmpeg-loglevel", default="warning")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    worker = StreamWorker(args)
    shutdown = threading.Event()

    def request_shutdown(_signum, _frame) -> None:
        shutdown.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    host, port = args.listen
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        server.settimeout(0.5)
        LOG.info("control server listening on %s:%d", host, port)
        while not shutdown.is_set():
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue
            LOG.info("Quest control connection from %s:%d", *address)
            with conn:
                conn.settimeout(1.0)
                while not shutdown.is_set():
                    try:
                        message = receive_command(conn)
                    except socket.timeout:
                        continue
                    except (EOFError, OSError, ValueError) as exc:
                        LOG.warning("control connection ended: %s", exc)
                        break
                    if message is None:
                        break
                    command, payload = message
                    LOG.info("control command %s (%d bytes)", command, len(payload))
                    if command == "OPEN_CAMERA":
                        request = parse_camera_request(payload)
                        LOG.info("camera request: %s", request)
                        worker.start(request, address[0])
                    elif command == "CLOSE_CAMERA":
                        worker.stop()
                    else:
                        LOG.warning("unsupported command: %s", command)
            worker.stop()
            LOG.info("Quest control connection closed")
    worker.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

