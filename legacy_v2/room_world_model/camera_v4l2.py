"""Small Linux V4L2 single-planar MMAP reader with real DQBUF metadata.

Supported: 64-bit Linux x86_64/aarch64, UVC VIDEO_CAPTURE, MJPG or YUYV.
Unsupported formats/devices fail explicitly; there is no silent OpenCV fallback.
The structs follow linux/videodev2.h.  tests/test_v4l2_abi.py checks them against
the installed kernel UAPI header when a C compiler is available.

https://docs.kernel.org/userspace-api/media/v4l/mmap.html
https://docs.kernel.org/userspace-api/media/v4l/vidioc-qbuf.html
"""
from __future__ import annotations

import ctypes as C
from dataclasses import dataclass
import errno
from fractions import Fraction
import math
import mmap
import os
import platform
import select
import time

from .timebase import TIMESTAMP_MASK, TIMESTAMP_MONOTONIC

U8, U32, I32 = C.c_uint8, C.c_uint32, C.c_int32


class Capability(C.Structure):
    _fields_ = [("driver", U8 * 16), ("card", U8 * 32), ("bus_info", U8 * 32),
                ("version", U32), ("capabilities", U32), ("device_caps", U32), ("reserved", U32 * 3)]


class PixFormat(C.Structure):
    _fields_ = [(name, U32) for name in ("width", "height", "pixelformat", "field", "bytesperline",
                 "sizeimage", "colorspace", "priv", "flags", "ycbcr_enc", "quantization", "xfer_func")]


class FormatUnion(C.Union):
    # v4l2_window contains a pointer: preserve its native union alignment.
    _fields_ = [("pix", PixFormat), ("raw_data", U8 * 200), ("alignment", C.c_void_p)]


class Format(C.Structure):
    _fields_ = [("type", U32), ("fmt", FormatUnion)]


class RequestBuffers(C.Structure):
    _fields_ = [("count", U32), ("type", U32), ("memory", U32), ("capabilities", U32),
                ("flags", U8), ("reserved", U8 * 3)]


class TimeVal(C.Structure):
    _fields_ = [("tv_sec", C.c_long), ("tv_usec", C.c_long)]


class TimeCode(C.Structure):
    _fields_ = [("type", U32), ("flags", U32), ("frames", U8), ("seconds", U8),
                ("minutes", U8), ("hours", U8), ("userbits", U8 * 4)]


class BufferMemory(C.Union):
    _fields_ = [("offset", U32), ("userptr", C.c_ulong), ("planes", C.c_void_p), ("fd", I32)]


class Buffer(C.Structure):
    _fields_ = [("index", U32), ("type", U32), ("bytesused", U32), ("flags", U32), ("field", U32),
                ("timestamp", TimeVal), ("timecode", TimeCode), ("sequence", U32), ("memory", U32),
                ("m", BufferMemory), ("length", U32), ("reserved2", U32), ("request_fd", I32)]


class FractionU32(C.Structure):
    _fields_ = [("numerator", U32), ("denominator", U32)]


class CaptureParm(C.Structure):
    _fields_ = [("capability", U32), ("capturemode", U32), ("timeperframe", FractionU32),
                ("extendedmode", U32), ("readbuffers", U32), ("reserved", U32 * 4)]


class ParmUnion(C.Union):
    _fields_ = [("capture", CaptureParm), ("raw_data", U8 * 200)]


class StreamParm(C.Structure):
    _fields_ = [("type", U32), ("parm", ParmUnion)]


class Control(C.Structure):
    _fields_ = [("id", U32), ("value", I32)]


def _ioc(direction: int, number: int, cls) -> int:
    return (direction << 30) | (C.sizeof(cls) << 16) | (ord("V") << 8) | number


VIDIOC_QUERYCAP = _ioc(2, 0, Capability)
VIDIOC_G_FMT = _ioc(3, 4, Format)
VIDIOC_S_FMT = _ioc(3, 5, Format)
VIDIOC_REQBUFS = _ioc(3, 8, RequestBuffers)
VIDIOC_QUERYBUF = _ioc(3, 9, Buffer)
VIDIOC_QBUF = _ioc(3, 15, Buffer)
VIDIOC_DQBUF = _ioc(3, 17, Buffer)
VIDIOC_STREAMON = _ioc(1, 18, C.c_int)
VIDIOC_STREAMOFF = _ioc(1, 19, C.c_int)
VIDIOC_G_PARM = _ioc(3, 21, StreamParm)
VIDIOC_S_PARM = _ioc(3, 22, StreamParm)
VIDIOC_G_CTRL = _ioc(3, 27, Control)
VIDIOC_S_CTRL = _ioc(3, 28, Control)
VIDEO_CAPTURE, MEMORY_MMAP = 1, 1
BUF_ERROR = 0x00000040
CONTROL_IDS = {
    "exposure_auto": 0x009A0901, "exposure_auto_priority": 0x009A0903,
    "exposure_absolute": 0x009A0902, "focus_auto": 0x009A090C, "focus_absolute": 0x009A090A,
    "autogain": 0x00980912, "gain": 0x00980913,
    "auto_white_balance": 0x0098090C, "white_balance_temperature": 0x0098091A,
}


def ioctl_struct(fd: int, request: int, value) -> None:
    import fcntl
    payload = bytearray(C.string_at(C.addressof(value), C.sizeof(value)))
    while True:
        try:
            fcntl.ioctl(fd, request, payload, True)
            C.memmove(C.addressof(value), bytes(payload), len(payload))
            return
        except OSError as exc:
            if exc.errno != errno.EINTR:
                raise


def fourcc(text: str) -> int:
    if len(text) != 4 or not text.isascii():
        raise ValueError("Pixel format must be a four-character ASCII V4L2 code")
    return int.from_bytes(text.encode("ascii"), "little")


@dataclass(frozen=True)
class RawFrame:
    payload: bytes
    dequeue_mono_ns: int
    timestamp_ns: int
    flags: int
    sequence: int
    skipped_ready_buffers: int = 0


class V4L2Camera:
    def __init__(self, device: str | int = "/dev/video0", *, width: int = 640, height: int = 480,
                 fps: float = 15.0, pixel_format: str = "MJPG", buffer_count: int = 4,
                 controls: dict[str, int] | None = None):
        if platform.system() != "Linux" or C.sizeof(C.c_void_p) != 8 or platform.machine().lower() not in {
            "x86_64", "amd64", "aarch64", "arm64"}:
            raise RuntimeError("Direct V4L2 backend requires 64-bit Linux x86_64/aarch64 (Pi OS/Ubuntu 64-bit)")
        if type(width) is not int or type(height) is not int or not 16 <= width <= 1920 or not 16 <= height <= 1080:
            raise ValueError("Camera size must be integer dimensions between 16 and 1920x1080")
        if isinstance(fps, bool) or not math.isfinite(fps) or fps <= 0 or fps > 240:
            raise ValueError("Camera FPS must be finite and in (0,240]")
        if pixel_format not in {"MJPG", "YUYV"}:
            raise ValueError("Only MJPG and YUYV single-planar capture is implemented")
        if pixel_format == "YUYV" and width % 2:
            raise ValueError("YUYV requires an even width")
        if type(buffer_count) is not int or not 2 <= buffer_count <= 16:
            raise ValueError("buffer_count must be 2..16")
        controls = {} if controls is None else controls
        if not isinstance(controls, dict) or set(controls) - set(CONTROL_IDS):
            raise ValueError(f"Camera controls must use supported names: {sorted(CONTROL_IDS)}")
        if any(type(x) is not int or not -(2**31) <= x < 2**31 for x in controls.values()):
            raise ValueError("V4L2 control settings must be integer driver units")
        self.device = f"/dev/video{device}" if isinstance(device, int) or str(device).isdigit() else str(device)
        self.width, self.height, self.pixel_format = width, height, pixel_format
        self.fd = None
        self.buffers: list[mmap.mmap] = []
        self.streaming = False
        self.corrupt_buffers = 0
        self.dropped_ready_buffers = 0
        self._last_sequence = None
        self._last_monotonic_stamp = None
        self.metadata = {}
        try:
            self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK | os.O_CLOEXEC)
            caps = Capability()
            ioctl_struct(self.fd, VIDIOC_QUERYCAP, caps)
            effective_caps = caps.device_caps if caps.capabilities & 0x80000000 else caps.capabilities
            if not effective_caps & 0x00000001 or not effective_caps & 0x04000000:
                raise RuntimeError("Camera must support V4L2 VIDEO_CAPTURE and STREAMING single-planar MMAP")
            fmt = Format(type=VIDEO_CAPTURE)
            fmt.fmt.pix.width, fmt.fmt.pix.height = width, height
            fmt.fmt.pix.pixelformat = fourcc(pixel_format)
            fmt.fmt.pix.field = 1  # V4L2_FIELD_NONE, progressive full frames only.
            ioctl_struct(self.fd, VIDIOC_S_FMT, fmt)
            actual = fmt.fmt.pix
            if (actual.width, actual.height, actual.pixelformat) != (width, height, fourcc(pixel_format)):
                raise RuntimeError("Driver changed the requested camera size/format; recalibrate that exact mode")
            if actual.field != 1:
                raise RuntimeError("Only progressive V4L2_FIELD_NONE frames are supported")
            self.bytesperline = max(width * 2, int(actual.bytesperline)) if pixel_format == "YUYV" else 0
            period = Fraction(1 / fps).limit_denominator(1_000_000)
            parm = StreamParm(type=VIDEO_CAPTURE)
            parm.parm.capture.timeperframe.numerator = period.numerator
            parm.parm.capture.timeperframe.denominator = period.denominator
            fps_set_error = None
            try:
                ioctl_struct(self.fd, VIDIOC_S_PARM, parm)
            except OSError as exc:
                if exc.errno not in {errno.EINVAL, errno.ENOTTY}:
                    raise
                fps_set_error = str(exc)
                # A driver without S_PARM may still provide a fixed mode via G_PARM.
                parm = StreamParm(type=VIDEO_CAPTURE)
                try:
                    ioctl_struct(self.fd, VIDIOC_G_PARM, parm)
                except OSError as read_exc:
                    if read_exc.errno not in {errno.EINVAL, errno.ENOTTY}:
                        raise
            actual_period = parm.parm.capture.timeperframe
            actual_fps = (actual_period.denominator / actual_period.numerator) if actual_period.numerator else None
            if actual_fps is not None and abs(actual_fps - fps) > max(0.02, 0.01 * fps):
                raise RuntimeError(f"Driver negotiated {actual_fps:g} FPS, requested {fps:g}; calibrate its actual mode")
            # Auto controls first, manual values afterwards; no implicit writes.
            for name, identifier in CONTROL_IDS.items():
                if name in controls:
                    control = Control(id=identifier, value=controls[name])
                    ioctl_struct(self.fd, VIDIOC_S_CTRL, control)
            readback = {}
            for name, identifier in CONTROL_IDS.items():
                control = Control(id=identifier)
                try:
                    ioctl_struct(self.fd, VIDIOC_G_CTRL, control)
                    readback[name] = int(control.value)
                except OSError as exc:
                    if exc.errno not in {errno.EINVAL, errno.ENOTTY} or name in controls:
                        raise
                if name in controls and readback.get(name) != controls[name]:
                    raise RuntimeError(f"Driver did not retain camera control {name}; recalibrate actual settings")
            req = RequestBuffers(count=buffer_count, type=VIDEO_CAPTURE, memory=MEMORY_MMAP)
            ioctl_struct(self.fd, VIDIOC_REQBUFS, req)
            if not 2 <= req.count <= 32:
                raise RuntimeError("V4L2 returned an unsupported number of streaming buffers")
            for index in range(req.count):
                buf = Buffer(index=index, type=VIDEO_CAPTURE, memory=MEMORY_MMAP)
                ioctl_struct(self.fd, VIDIOC_QUERYBUF, buf)
                if not 0 < buf.length <= 32_000_000:
                    raise RuntimeError("V4L2 returned an invalid image buffer length")
                self.buffers.append(mmap.mmap(self.fd, buf.length, flags=mmap.MAP_SHARED,
                                             prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset))
                self._requeue(index)
            ioctl_struct(self.fd, VIDIOC_STREAMON, C.c_int(VIDEO_CAPTURE))
            self.streaming = True
            decode = lambda x: bytes(x).split(b"\0", 1)[0].decode("utf-8", errors="replace")
            self.metadata = {"backend": "v4l2_mmap", "device": self.device, "driver": decode(caps.driver),
                "card": decode(caps.card), "bus_info": decode(caps.bus_info), "width": width, "height": height,
                "pixel_format": pixel_format, "requested_fps": fps, "reported_fps": actual_fps,
                "fps_set_error": fps_set_error, "bytesperline": self.bytesperline, "buffer_count": req.count,
                "configured_controls": dict(controls), "controls_readback": readback}
        except BaseException:
            self.close()
            raise

    def _requeue(self, index: int) -> None:
        buf = Buffer(index=index, type=VIDEO_CAPTURE, memory=MEMORY_MMAP)
        ioctl_struct(self.fd, VIDIOC_QBUF, buf)

    def _dequeue_one(self) -> RawFrame | None:
        buf = Buffer(type=VIDEO_CAPTURE, memory=MEMORY_MMAP)
        try:
            ioctl_struct(self.fd, VIDIOC_DQBUF, buf)
        except OSError as exc:
            if exc.errno == errno.EAGAIN:
                return None
            raise
        # Read immediately after the actual ioctl, before copying/decompression.
        dequeue_ns = time.monotonic_ns()
        if buf.index >= len(self.buffers):
            raise RuntimeError("V4L2 returned an invalid buffer index")
        try:
            if buf.bytesused > len(self.buffers[buf.index]):
                raise RuntimeError("V4L2 payload exceeds its MMAP buffer")
            if buf.flags & BUF_ERROR or not buf.bytesused:
                self.corrupt_buffers += 1
                return RawFrame(b"", dequeue_ns, 0, int(buf.flags), int(buf.sequence))
            if buf.timestamp.tv_sec < 0 or not 0 <= buf.timestamp.tv_usec < 1_000_000:
                raise RuntimeError("V4L2 returned an invalid timeval timestamp")
            stamp = buf.timestamp.tv_sec * 1_000_000_000 + buf.timestamp.tv_usec * 1_000
            if self._last_sequence is not None:
                delta = (int(buf.sequence) - self._last_sequence) & 0xFFFFFFFF
                if delta == 0 or delta >= 2**31:
                    raise RuntimeError("V4L2 sequence repeated/reset: restart the camera session explicitly")
            self._last_sequence = int(buf.sequence)
            if buf.flags & TIMESTAMP_MASK == TIMESTAMP_MONOTONIC and stamp > 0:
                if self._last_monotonic_stamp is not None and stamp <= self._last_monotonic_stamp:
                    raise RuntimeError("V4L2 monotonic timestamp stopped/regressed")
                self._last_monotonic_stamp = stamp
            payload = self.buffers[buf.index][:buf.bytesused]
            return RawFrame(payload, dequeue_ns, stamp, int(buf.flags), int(buf.sequence))
        finally:
            # Camera gets the memory back before user code decodes or infers.
            self._requeue(int(buf.index))

    def dequeue_latest(self, timeout_s: float = 0.25) -> RawFrame | None:
        """Drain ready buffers and return only the newest payload, without decode.

        A finite drain budget avoids starvation if a broken source produces data
        without waiting. The caller continuously repeats this in its own thread.
        """
        if self.fd is None:
            raise RuntimeError("Camera is closed")
        ready, _, errors = select.select([self.fd], [], [self.fd], timeout_s)
        if errors:
            raise RuntimeError("V4L2 camera poll error")
        if not ready:
            return None
        latest = None
        drained = 0
        for _ in range(max(2, len(self.buffers) * 2)):
            frame = self._dequeue_one()
            if frame is None:
                break
            if not frame.payload:
                continue
            drained += 1
            latest = frame
        if latest is None:
            return None
        self.dropped_ready_buffers += max(0, drained - 1)
        return RawFrame(latest.payload, latest.dequeue_mono_ns, latest.timestamp_ns, latest.flags,
                        latest.sequence, max(0, drained - 1))

    def decode(self, raw: RawFrame):
        import cv2
        import numpy as np
        if self.pixel_format == "MJPG":
            image = cv2.imdecode(np.frombuffer(raw.payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        else:
            required = self.height * self.bytesperline
            if len(raw.payload) < required:
                raise RuntimeError("Truncated YUYV frame")
            rows = np.frombuffer(raw.payload, dtype=np.uint8, count=required).reshape(self.height, self.bytesperline)
            packed = rows[:, :self.width * 2].copy().reshape(self.height, self.width, 2)
            image = cv2.cvtColor(packed, cv2.COLOR_YUV2BGR_YUYV)
        if image is None or image.shape != (self.height, self.width, 3):
            raise RuntimeError("Camera image cannot be decoded at the calibrated resolution")
        return image

    def close(self) -> None:
        if self.fd is None:
            return
        if self.streaming:
            try:
                ioctl_struct(self.fd, VIDIOC_STREAMOFF, C.c_int(VIDEO_CAPTURE))
            except OSError:
                pass
            self.streaming = False
        for mapping in self.buffers:
            mapping.close()
        self.buffers.clear()
        os.close(self.fd)
        self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
