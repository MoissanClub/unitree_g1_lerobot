"""Bounded, same-host RGB frame exchange, independent of MuJoCo, DDS, and XR.

A memory-mapped latest-frame file uses advisory locks to avoid torn reads. Readers
and writers never wait for each other; busy frames are skipped. A separate owner
lock prevents competing producers. Readers reopen the path to survive restarts.
"""
import fcntl
import json
import mmap
import os
from pathlib import Path
import struct
import time
import uuid

import numpy as np

HEADER_BYTES = 4096
VERSION = 1


def default_channel():
    return Path(f"/tmp/lerobot-camera-{os.getuid()}.rgb")


class FrameWriter:
    def __init__(self, path, width, height):
        self.path = Path(path)
        self.width, self.height = width, height
        self.owner = self.fd = self.mapping = None
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + "." + self.session_id)
        try:
            self.owner = os.open(str(self.path) + ".owner", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # New inode on restart: never truncate an old reader's mapped file.
            self.fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            os.ftruncate(self.fd, HEADER_BYTES + width * height * 3)
            self.mapping = mmap.mmap(self.fd, 0)
            self.mapping[:HEADER_BYTES] = bytes(HEADER_BYTES)
            os.replace(temporary, self.path)
        except BaseException:
            self.close(unlink=False)
            temporary.unlink(missing_ok=True)
            raise

    def publish(self, pixels, metadata):
        if pixels.shape != (self.height, self.width, 3) or pixels.dtype != np.uint8:
            raise ValueError("Expected HxWx3 uint8 RGB pixels")
        info = dict(metadata, version=VERSION, pixel_format="RGB8", width=self.width,
                    height=self.height, sequence=self.sequence + 1, session_id=self.session_id,
                    published_monotonic_ns=time.monotonic_ns())
        header = json.dumps(info, allow_nan=False).encode("utf8")
        if len(header) > HEADER_BYTES - 4:
            raise ValueError("Camera metadata too large")
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        try:
            self.mapping[HEADER_BYTES:] = pixels.tobytes()
            self.mapping[4:4+len(header)] = header
            self.mapping[:4] = struct.pack("<I", len(header))
            self.sequence += 1
        finally:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
        return True

    def close(self, unlink=True):
        if self.mapping is not None:
            self.mapping.close()
            self.mapping = None
        if self.fd is not None:
            if unlink:
                self.path.unlink(missing_ok=True)
            os.close(self.fd)
            self.fd = None
        if self.owner is not None:
            os.close(self.owner)
            self.owner = None


def read_frame(path, max_age_s=1.0):
    """Return owned (metadata, RGB pixels), or None if missing, busy, or stale."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    except FileNotFoundError:
        return None
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            return None
        size = os.fstat(fd).st_size
        if size < HEADER_BYTES:
            return None
        with mmap.mmap(fd, 0, access=mmap.ACCESS_READ) as mapping:
            length, = struct.unpack("<I", mapping[:4])
            if not length:
                return None
            if length > HEADER_BYTES - 4:
                raise ValueError("Invalid camera header")
            info = json.loads(mapping[4:4+length])
            if info["version"] != VERSION or info["pixel_format"] != "RGB8":
                raise ValueError("Unsupported camera frame protocol")
            width, height = info["width"], info["height"]
            if min(width, height) <= 0 or size != HEADER_BYTES + width * height * 3:
                raise ValueError("Invalid camera frame dimensions")
            age = (time.monotonic_ns() - info["captured_monotonic_ns"]) / 1e9
            if age < 0 or age > max_age_s:
                return None
            pixels = np.frombuffer(mapping[HEADER_BYTES:], dtype=np.uint8).reshape(height, width, 3)
            return info, pixels
    finally:
        os.close(fd)
