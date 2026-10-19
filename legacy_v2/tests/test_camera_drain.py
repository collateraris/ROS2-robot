"""An inference/decode stall must not stop device draining or replay stale FIFO."""
import queue
import threading
import time

import numpy as np

from room_world_model.camera import LatestCamera
from room_world_model.camera_v4l2 import RawFrame


class ControlledCapture:
    def __init__(self):
        self.input = queue.Queue()
        self.decoding_first = threading.Event()
        self.allow_decode = threading.Event()
        self.drained_third = threading.Event()
        self.decoded_third = threading.Event()
        self.closed = False

    def dequeue_latest(self, timeout_s):
        try:
            item = self.input.get(timeout=timeout_s)
        except queue.Empty:
            return None
        if item.sequence == 3:
            self.drained_third.set()
        return item

    def decode(self, raw):
        if raw.sequence == 1:
            self.decoding_first.set()
            assert self.allow_decode.wait(2)
        if raw.sequence == 3:
            self.decoded_third.set()
        return np.full((16, 24, 3), raw.sequence, np.uint8)

    def close(self):
        self.closed = True

    def push(self, sequence):
        self.input.put(RawFrame(b"payload", time.monotonic_ns(), 0, 0, sequence))


def test_slow_decode_does_not_block_capture_and_newest_pending_replaces_old():
    capture = ControlledCapture()
    camera = LatestCamera({"device": 0, "width": 24, "height": 16, "requested_fps": 15}, capture=capture)
    try:
        capture.push(1)
        assert capture.decoding_first.wait(2)
        capture.push(2)
        capture.push(3)
        assert capture.drained_third.wait(2), "Camera acquisition must continue while JPEG decoding is blocked"
        capture.allow_decode.set()
        deadline = time.monotonic() + 2
        sample = None
        while time.monotonic() < deadline:
            candidate = camera.take_latest()
            if candidate is not None and candidate.sequence == 3:
                sample = candidate
                break
            threading.Event().wait(.005)
        assert sample is not None and sample.bgr[0, 0, 0] == 3
        assert camera.take_latest() is None
        assert not sample.timing.timing_valid
        assert camera.metadata["undecoded_frames_replaced"] >= 1
    finally:
        capture.allow_decode.set()
        camera.close()
    assert capture.closed and not camera._thread.is_alive()
