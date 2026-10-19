"""Continuously drained acquisition with a depth-one decoded mailbox.

The Linux backend reads VIDIOC_DQBUF metadata. OpenCV is an explicit empirical
fallback; read duration never means exposure age. Capture and image decoding
run independently so a slow inference consumer does not stop the queue drain.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time
import uuid

from .timebase import CaptureTiming, TimingConfig, exposure_bounds


@dataclass(frozen=True)
class CameraSample:
    bgr: object
    read_mono_s: float
    read_duration_s: float
    sequence: int
    epoch: str
    timing: CaptureTiming
    ready_mono_s: float
    skipped_ready_buffers: int = 0

    @property
    def exposure_earliest_ns(self):
        return self.timing.exposure_earliest_ns

    @property
    def exposure_latest_ns(self):
        return self.timing.exposure_latest_ns

    @property
    def timestamp_source(self):
        return self.timing.timestamp_source


class LatestCamera:
    """Own one camera and deliver each newest decoded frame at most once.

    camera.backend = 'v4l2' (default), or explicit 'opencv'. V4L2 settings:
    device, width, height, requested_fps, pixel_format='MJPG', buffer_count=4.
    camera.timing is a TimingConfig mapping. Omitted timing is uncalibrated;
    the v1 capture_latency_* placeholder configuration is deliberately ignored.
    """
    def __init__(self, config: dict, capture=None):
        self.config = config
        self.timing_config = TimingConfig.from_dict(config.get("timing"))
        self.backend = config.get("backend", "v4l2")
        if self.backend not in {"v4l2", "opencv"}:
            raise ValueError("camera.backend must be v4l2 or explicit opencv")
        self.stall_timeout_s = config.get("stall_timeout_s", 2.0)
        if isinstance(self.stall_timeout_s, bool) or not math.isfinite(self.stall_timeout_s) or self.stall_timeout_s < 0.5:
            raise ValueError("camera.stall_timeout_s must be finite and at least 0.5 s")
        if self.backend == "v4l2":
            from .camera_v4l2 import V4L2Camera
            self.capture = capture if capture is not None else V4L2Camera(config["device"],
                width=config["width"], height=config["height"], fps=config["requested_fps"],
                pixel_format=config.get("pixel_format", "MJPG"), buffer_count=config.get("buffer_count", 4),
                controls=config.get("controls"))
        else:
            if config.get("controls"):
                raise ValueError("Explicit V4L2 controls require backend=v4l2; OpenCV uses different control semantics")
            import cv2
            device = config["device"]
            if isinstance(device, str) and device.isdigit():
                device = int(device)
            self.capture = capture if capture is not None else cv2.VideoCapture(device)
            if not self.capture.isOpened():
                self.capture.release()
                raise RuntimeError("Cannot open camera; stop the previous owner first")
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.get("pixel_format", "MJPG")))
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
            self.capture.set(cv2.CAP_PROP_FPS, config["requested_fps"])
            # May be ignored; this property never establishes freshness.
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.epoch = uuid.uuid4().hex
        self.error: str | None = None
        self._lock = threading.Lock()
        self._sample = self._raw = None
        self._raw_ready = threading.Event()
        self._stop = threading.Event()
        self._decoded_dropped = self._raw_dropped = 0
        self._decoder = None
        if self.backend == "v4l2":
            self._decoder = threading.Thread(target=self._decode_run, daemon=True, name="camera-decode-latest")
            self._decoder.start()
        target = self._run_v4l2 if self.backend == "v4l2" else self._run_opencv
        self._thread = threading.Thread(target=target, daemon=True, name="camera-drain-latest")
        self._thread.start()

    @property
    def metadata(self) -> dict:
        return {**getattr(self.capture, "metadata", {"backend": self.backend}),
                "camera_session_id": self.epoch, "unconsumed_decoded_frames_replaced": self._decoded_dropped,
                "undecoded_frames_replaced": self._raw_dropped}

    def _publish(self, sample: CameraSample):
        if sample.bgr.shape[:2] != (self.config["height"], self.config["width"]):
            raise RuntimeError("camera_resolution_differs_from_calibration")
        with self._lock:
            if self._sample is not None:
                self._decoded_dropped += 1
            self._sample = sample

    def _run_v4l2(self):
        seq, last_frame = 0, time.monotonic()
        try:
            while not self._stop.is_set():
                start = time.monotonic()
                raw = self.capture.dequeue_latest(timeout_s=0.25)
                if raw is None:
                    if time.monotonic() - last_frame > self.stall_timeout_s:
                        raise RuntimeError("camera_dequeue_timeout")
                    continue
                last_frame = time.monotonic()
                timing = exposure_bounds(dequeue_mono_ns=raw.dequeue_mono_ns,
                    v4l2_timestamp_ns=raw.timestamp_ns, v4l2_flags=raw.flags,
                    hardware_sequence=raw.sequence, config=self.timing_config)
                seq += 1
                with self._lock:
                    if self._raw is not None:
                        self._raw_dropped += 1
                    self._raw = (raw, timing, seq, last_frame - start)
                    self._raw_ready.set()
        except Exception as exc:
            self.error = f"camera_exception: {exc}"
            self._stop.set()
            self._raw_ready.set()
        finally:
            self.capture.close()

    def _decode_run(self):
        try:
            while not self._stop.is_set():
                self._raw_ready.wait(0.25)
                with self._lock:
                    pending, self._raw = self._raw, None
                    self._raw_ready.clear()
                if pending is None:
                    continue
                raw, timing, seq, duration = pending
                bgr = self.capture.decode(raw)
                self._publish(CameraSample(bgr, raw.dequeue_mono_ns / 1e9, duration, seq,
                    self.epoch, timing, time.monotonic(), raw.skipped_ready_buffers))
        except Exception as exc:
            self.error = f"camera_decode_exception: {exc}"
            self._stop.set()

    def _run_opencv(self):
        seq = 0
        try:
            while not self._stop.is_set():
                start_ns = time.monotonic_ns()
                ok, bgr = self.capture.read()
                now_ns = time.monotonic_ns()
                if not ok or bgr is None:
                    raise RuntimeError("camera_read_failed")
                seq += 1
                timing = exposure_bounds(dequeue_mono_ns=now_ns, config=self.timing_config)
                self._publish(CameraSample(bgr, now_ns / 1e9, (now_ns - start_ns) / 1e9,
                    seq, self.epoch, timing, now_ns / 1e9))
                if now_ns - start_ns < 1_000_000:
                    self._stop.wait(0.001)
        except Exception as exc:
            self.error = f"camera_exception: {exc}"
            self._stop.set()
        finally:
            self.capture.release()

    def take_latest(self):
        with self._lock:
            sample, self._sample = self._sample, None
            return sample

    def close(self):
        self._stop.set()
        self._raw_ready.set()
        self._thread.join(timeout=1.0)
        if self._decoder is not None:
            self._decoder.join(timeout=1.0)
        # Do not release a backend concurrently with blocked read. The V4L2
        # reader has bounded polling; an OpenCV fallback may remain blocked in
        # its daemon thread, which cannot keep the motor safety timer alive.
