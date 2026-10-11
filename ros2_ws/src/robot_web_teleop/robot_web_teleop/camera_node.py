"""UVC camera: strict control images and optional slow-frame JPEG preview.

All header stamps are host receipt times, never hardware exposure timestamps.
"""
import math
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        defaults = dict(device='/dev/video0', width=640, height=480, fps=15.0,
                        jpeg_quality=70, preview_enabled=False, pixel_format='MJPG')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.device = str(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.quality = int(self.get_parameter('jpeg_quality').value)
        self.preview_enabled = self.get_parameter('preview_enabled').value
        self.pixel_format = str(self.get_parameter('pixel_format').value).upper()
        if (not self.device or not 16 <= self.width <= 1920
                or not 16 <= self.height <= 1080 or not math.isfinite(self.fps)
                or not 1 <= self.fps <= 60 or not 1 <= self.quality <= 100
                or self.pixel_format not in ('MJPG', 'YUYV')
                or type(self.preview_enabled) is not bool):
            raise ValueError('Invalid camera device, dimensions, fps, JPEG quality or format')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.raw_pub = self.create_publisher(Image, 'camera/image_raw', qos)
        self.jpeg_pub = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos)
        self.preview_pub = (self.create_publisher(CompressedImage, 'camera/preview/compressed', qos)
                            if self.preview_enabled else None)
        self.bridge = CvBridge()
        self.lock, self.stopping = threading.Lock(), threading.Event()
        self.latest = self.preview_latest = None
        self.error = ''
        self.sequence = self.published = self.preview_published = 0
        self.last_warning = self.last_slow_warning = float('-inf')
        self.worker = threading.Thread(target=self.capture, name='uvc-capture', daemon=True)
        self.timer = self.create_timer(1 / self.fps, self.publish_latest)
        self.worker.start()

    def capture(self):
        # Only this thread touches cap. V4L2 does not support OpenCV's
        # READ_TIMEOUT_MSEC; a kernel/USB hang must not hold the ROS executor.
        while not self.stopping.is_set():
            cap = cv2.VideoCapture()
            try:
                if not cap.open(self.device, cv2.CAP_V4L2):
                    raise RuntimeError(f'Cannot open {self.device} with V4L2')
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Driver-dependent best effort.
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
                fourcc = cap.get(cv2.CAP_PROP_FOURCC)
                actual_format = (''.join(chr((int(fourcc) >> (8 * i)) & 255) for i in range(4))
                                 if math.isfinite(fourcc) and fourcc > 0 else 'unknown')
                self.get_logger().info(
                    f'V4L2 selected {actual_format!r}, '
                    f'{cap.get(cv2.CAP_PROP_FRAME_WIDTH):g}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):g}, '
                    f'{cap.get(cv2.CAP_PROP_FPS):g} fps reported; requested '
                    f'{self.pixel_format} {self.width}x{self.height} at {self.fps:g} fps. '
                    'Image stamps are host receipt time, not exposure time.')
                while not self.stopping.is_set():
                    started = time.monotonic()
                    ok, frame = cap.read()
                    received = time.monotonic()
                    if self.stopping.is_set():
                        break
                    if not ok or frame is None or not frame.size:
                        raise RuntimeError('Camera read failed or returned an empty frame; reconnecting')
                    # This is host receipt time, not the hardware exposure time.
                    stamp = self.get_clock().now().to_msg()
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    elif frame.ndim == 3 and frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    if frame.ndim != 3 or frame.shape[2] != 3 or str(frame.dtype) != 'uint8':
                        raise RuntimeError('V4L2 did not return an 8-bit BGR image')
                    elapsed = received - started
                    control_ok = 0 <= elapsed <= max(.5, 2 / self.fps)
                    with self.lock:
                        self.sequence += 1
                        item = self.sequence, frame, stamp, received
                        self.latest = item if control_ok else None
                        self.preview_latest = item if self.preview_enabled else None
                        self.error = ''
                    if not control_ok and received - self.last_slow_warning >= 5:
                        self.get_logger().warning(
                            f'slow successful read {elapsed:.3f} seconds, control frames withheld')
                        self.last_slow_warning = received
            except (cv2.error, RuntimeError, ValueError) as error:
                with self.lock:
                    self.latest = self.preview_latest = None
                    self.error = str(error)
            finally:
                cap.release()
            self.stopping.wait(1.0)

    def publish_latest(self):
        now = time.monotonic()
        with self.lock:
            error = self.error
            pending = {}
            for item, counter, stream in ((self.latest, 'published', 1),
                                         (self.preview_latest, 'preview_published', 2)):
                if (item is not None and item[0] != getattr(self, counter)
                        and 0 <= now - item[3] <= .5):
                    pending.setdefault(item[0], [item, False, False])[stream] = True
                    setattr(self, counter, item[0])
        if error and now - self.last_warning > 5:
            self.get_logger().warning(error)
            self.last_warning = now
        for item, control, preview in pending.values():
            sequence, frame, stamp, received = item
            try:
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    raise RuntimeError('JPEG encoding failed')
                compressed = CompressedImage()
                compressed.header.stamp, compressed.header.frame_id = stamp, 'camera_optical_frame'
                compressed.format, compressed.data = 'bgr8; jpeg compressed bgr8', jpeg.tobytes()
                if control:
                    raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    raw.header = compressed.header
                # Encoding may have been slow, or capture may have invalidated this item.
                # Check again before publication; never manufacture a fresh stamp here.
                with self.lock:
                    if not 0 <= time.monotonic() - received <= .5:
                        continue
                    if control and self.latest is not None and self.latest[0] == sequence:
                        self.raw_pub.publish(raw)
                        self.jpeg_pub.publish(compressed)
                    if (preview and self.preview_latest is not None
                            and self.preview_latest[0] == sequence):
                        self.preview_pub.publish(compressed)
            except (cv2.error, RuntimeError, ValueError) as error:
                self.get_logger().error(str(error))

    def close(self):
        self.timer.cancel()
        self.stopping.set()
        self.worker.join(timeout=1.0)
        # Never release cap concurrently with a blocked cap.read(). On a driver
        # hang the daemon thread exits with this process; no old frames repeat.


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
