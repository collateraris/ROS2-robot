"""Exercise the real camera class with scripted reads; no ROS, cv2 or hardware."""
import ast
from dataclasses import dataclass
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "ros2_ws" / "src"
          / "robot_web_teleop" / "robot_web_teleop" / "camera_node.py")


@dataclass
class Frame:
    label: str
    shape: tuple = (480, 640, 3)
    dtype: str = "uint8"
    size: int = 480 * 640 * 3

    @property
    def ndim(self):
        return len(self.shape)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class StopEvent:
    def __init__(self, harness):
        self.harness, self.stopped = harness, False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        if not self.stopped:
            self.harness.reconnect_waits.append(seconds)
            self.harness.now += seconds
        return self.stopped


class Capture:
    def __init__(self, harness, reads):
        self.harness, self.reads = harness, iter(reads)
        self.settings, self.open_calls, self.release_calls = {}, [], 0

    def open(self, device, backend):
        self.open_calls.append((device, backend))
        return True

    def set(self, prop, value):
        self.settings[prop] = value
        return True

    def get(self, prop):
        return self.settings.get(prop, 0)

    def read(self):
        h = self.harness
        # Run one executor tick after the preceding read has been processed.
        # No concurrent threads, real time or sleeps are needed for these tests.
        h.snapshots.append((h.node.latest, h.node.preview_latest))
        h.node.publish_latest()
        try:
            duration, ok, frame = next(self.reads)
        except StopIteration:
            h.node.stopping.set()
            return False, None
        h.now += duration
        if isinstance(frame, Exception):
            raise frame
        return ok, frame

    def release(self):
        self.release_calls += 1


class CameraHarness:
    """Load class methods from source and replace only external dependencies."""
    def __init__(self, plans=(), preview=True, fps=15.0, pixel_format="MJPG"):
        self.now = 10.0
        self.plans = iter(plans)
        self.captures, self.open_snapshots, self.snapshots = [], [], []
        self.reconnect_waits, self.encodes, self.raw_conversions = [], [], []
        self.warnings, self.errors, self.infos = [], [], []
        self.clock_calls = 0
        self.encode_hook = None
        self.cv = SimpleNamespace(
            VideoCapture=self.make_capture,
            VideoWriter_fourcc=lambda *chars: sum(ord(c) << (8 * i)
                                                 for i, c in enumerate(chars)),
            CAP_V4L2=1, CAP_PROP_FOURCC=2, CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5, CAP_PROP_BUFFERSIZE=6,
            CAP_PROP_CONVERT_RGB=7, COLOR_GRAY2BGR=8, COLOR_BGRA2BGR=9,
            IMWRITE_JPEG_QUALITY=10, error=type("CvError", (Exception,), {}),
            imencode=self.encode,
            cvtColor=lambda frame, code: Frame(frame.label),
        )
        namespace = dict(Node=object, cv2=self.cv, CompressedImage=Message, math=math,
                         time=SimpleNamespace(monotonic=lambda: self.now))
        tree = ast.parse(SOURCE.read_text())
        tree.body = [item for item in tree.body if isinstance(item, ast.ClassDef)]
        exec(compile(tree, str(SOURCE), "exec"), namespace)
        cls = namespace["CameraNode"]
        self.node = n = cls.__new__(cls)
        n.device, n.width, n.height = "/dev/video-test", 640, 480
        n.fps, n.quality, n.pixel_format = fps, 70, pixel_format
        n.preview_enabled = preview
        n.raw_pub, n.jpeg_pub = Publisher(), Publisher()
        n.preview_pub = Publisher() if preview else None
        n.bridge = SimpleNamespace(cv2_to_imgmsg=self.raw_message)
        n.lock, n.stopping = threading.Lock(), StopEvent(self)
        n.latest = n.preview_latest = None
        n.error = ""
        n.sequence = n.published = n.preview_published = 0
        n.last_warning, n.last_slow_warning = 0.0, float("-inf")
        n.get_clock = lambda: SimpleNamespace(now=self.ros_now)
        logger = SimpleNamespace(warning=self.warnings.append,
                                 error=self.errors.append, info=self.infos.append)
        n.get_logger = lambda: logger

    def make_capture(self):
        self.open_snapshots.append((self.node.latest, self.node.preview_latest,
                                    self.node.error))
        try:
            reads = next(self.plans)
        except StopIteration:
            raise AssertionError("Unexpected camera reopen")
        cap = Capture(self, reads)
        self.captures.append(cap)
        return cap

    def ros_now(self):
        self.clock_calls += 1
        nanoseconds = round(self.now * 1_000_000_000)
        stamp = SimpleNamespace(sec=nanoseconds // 1_000_000_000,
                                nanosec=nanoseconds % 1_000_000_000)
        return SimpleNamespace(to_msg=lambda: stamp)

    def raw_message(self, frame, encoding):
        self.raw_conversions.append((frame, encoding))
        message = Message()
        message.frame, message.encoding = frame, encoding
        return message

    def encode(self, extension, frame, options):
        self.encodes.append((extension, frame, options))
        if self.encode_hook is not None:
            self.encode_hook()
        return True, SimpleNamespace(tobytes=lambda: frame.label.encode("ascii"))

    def cache(self, sequence=1, age=0.0, strict=True, preview=True):
        stamp = SimpleNamespace(sec=123, nanosec=456)
        item = sequence, Frame(str(sequence)), stamp, self.now - age
        self.node.latest = item if strict else None
        self.node.preview_latest = item if preview else None
        return item


class CameraTests(unittest.TestCase):
    def test_reported_slow_reads_preview_only_then_fast_recovers_without_reopen(self):
        durations = (2.877, .567, .752, .030)
        h = CameraHarness([[(duration, True, Frame(str(i)))
                            for i, duration in enumerate(durations, 1)]])
        h.node.capture()
        n = h.node
        self.assertEqual(len(h.captures), 1)
        self.assertEqual(h.captures[0].release_calls, 1)
        self.assertEqual(h.reconnect_waits, [])
        self.assertEqual([m.data for m in n.preview_pub.messages],
                         [b"1", b"2", b"3", b"4"])
        self.assertEqual([m.data for m in n.jpeg_pub.messages], [b"4"])
        self.assertEqual([m.frame.label for m in n.raw_pub.messages], ["4"])
        self.assertEqual(len(h.encodes), 4)  # Fast frame serves both JPEG topics.
        self.assertEqual(len(h.raw_conversions), 1)
        for strict, preview in h.snapshots[1:4]:
            self.assertIsNone(strict)
            self.assertIsNotNone(preview)
        self.assertIs(n.jpeg_pub.messages[0].header.stamp,
                      n.preview_pub.messages[-1].header.stamp)
        self.assertIs(n.raw_pub.messages[0].header.stamp, n.latest[2])
        self.assertEqual(h.clock_calls, 4)
        self.assertEqual(n.error, "")
        self.assertTrue(any("slow successful read" in warning and
                            "control frames withheld" in warning
                            for warning in h.warnings))

    def test_preview_disabled_withholds_every_slow_frame(self):
        h = CameraHarness([[(duration, True, Frame("slow"))
                            for duration in (2.877, .567, .752)]], preview=False)
        h.node.capture()
        self.assertEqual(len(h.captures), 1)
        self.assertIsNone(h.node.latest)
        self.assertIsNone(h.node.preview_latest)
        self.assertIsNone(h.node.preview_pub)
        self.assertEqual(h.node.raw_pub.messages, [])
        self.assertEqual(h.node.jpeg_pub.messages, [])
        self.assertEqual(h.encodes, [])

    def test_slow_success_clears_previous_control_frame(self):
        h = CameraHarness([[(.03, True, Frame("fast")),
                            (.752, True, Frame("slow"))]])
        h.node.capture()
        self.assertIsNone(h.node.latest)
        self.assertEqual(h.node.preview_latest[1].label, "slow")
        self.assertEqual([m.data for m in h.node.jpeg_pub.messages], [b"fast"])
        self.assertEqual([m.data for m in h.node.preview_pub.messages],
                         [b"fast", b"slow"])

    def test_read_budget_respects_requested_fps_and_pixel_format(self):
        # At 2 fps, the allowed read time is 2 / fps = 1 second.
        h = CameraHarness([[(.752, True, Frame("within-budget"))]],
                          fps=2.0, pixel_format="YUYV")
        h.node.capture()
        self.assertEqual(len(h.node.raw_pub.messages), 1)
        cap = h.captures[0]
        self.assertEqual(cap.settings[h.cv.CAP_PROP_FOURCC],
                         h.cv.VideoWriter_fourcc(*"YUYV"))
        self.assertEqual(cap.open_calls, [(h.node.device, h.cv.CAP_V4L2)])

    def test_failed_empty_or_invalid_read_clears_both_caches_and_reconnects(self):
        failures = [
            (False, Frame("failed")), (True, None),
            (True, Frame("empty", size=0)),
            (True, Frame("invalid-shape", shape=(640,))),
            (True, Frame("invalid-dtype", dtype="float32")),
        ]
        for ok, frame in failures:
            with self.subTest(frame=frame, ok=ok):
                h = CameraHarness([
                    [(.03, True, Frame("before")), (.03, ok, frame)],
                    [(.03, True, Frame("after"))],
                ])
                h.node.capture()
                self.assertEqual(len(h.captures), 2)
                self.assertEqual([cap.release_calls for cap in h.captures], [1, 1])
                strict, preview, error = h.open_snapshots[1]
                self.assertIsNone(strict)
                self.assertIsNone(preview)
                self.assertTrue(error)
                self.assertEqual(h.reconnect_waits, [1.0])
                self.assertEqual([m.data for m in h.node.jpeg_pub.messages],
                                 [b"before", b"after"])

    def test_cached_frame_published_once_without_restamping(self):
        h = CameraHarness()
        item = h.cache(age=.5)
        h.node.publish_latest()
        h.now += .001
        h.node.publish_latest()
        self.assertEqual(len(h.encodes), 1)
        for pub in (h.node.raw_pub, h.node.jpeg_pub, h.node.preview_pub):
            self.assertEqual(len(pub.messages), 1)
            self.assertIs(pub.messages[0].header.stamp, item[2])
        self.assertEqual(h.clock_calls, 0)

    def test_stale_or_future_receipt_is_never_published(self):
        for age in (.501, -.001):
            with self.subTest(age=age):
                h = CameraHarness()
                h.cache(age=age)
                h.node.publish_latest()
                self.assertEqual(h.encodes, [])
                self.assertEqual(h.node.raw_pub.messages, [])
                self.assertEqual(h.node.jpeg_pub.messages, [])
                self.assertEqual(h.node.preview_pub.messages, [])
                self.assertEqual(h.clock_calls, 0)

    def test_valid_preview_is_independent_of_stale_control_cache(self):
        h = CameraHarness()
        stale = h.cache(sequence=1, age=.501)
        fresh = h.cache(sequence=2)
        h.node.latest = stale
        h.node.publish_latest()
        self.assertEqual(h.node.raw_pub.messages, [])
        self.assertEqual(h.node.jpeg_pub.messages, [])
        self.assertEqual([m.data for m in h.node.preview_pub.messages], [b"2"])
        self.assertIs(h.node.preview_pub.messages[0].header.stamp, fresh[2])

    def test_frame_that_expires_during_jpeg_encoding_is_withheld(self):
        h = CameraHarness()
        h.cache()
        h.encode_hook = lambda: setattr(h, "now", h.now + .501)
        h.node.publish_latest()
        self.assertEqual(len(h.encodes), 1)
        self.assertEqual(h.node.raw_pub.messages, [])
        self.assertEqual(h.node.jpeg_pub.messages, [])
        self.assertEqual(h.node.preview_pub.messages, [])

    def test_disconnect_during_jpeg_encoding_does_not_publish_old_cache(self):
        h = CameraHarness()
        h.cache()

        def disconnect():
            h.node.latest = h.node.preview_latest = None

        h.encode_hook = disconnect
        h.node.publish_latest()
        self.assertEqual(len(h.encodes), 1)
        self.assertEqual(h.node.raw_pub.messages, [])
        self.assertEqual(h.node.jpeg_pub.messages, [])
        self.assertEqual(h.node.preview_pub.messages, [])


if __name__ == "__main__":
    unittest.main()
