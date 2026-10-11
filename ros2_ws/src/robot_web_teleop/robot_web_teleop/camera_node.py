"""USB/UVC V4L2 camera: latest frame only; raw BGR8 and compressed JPEG."""
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
                        jpeg_quality=70)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.device = str(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.quality = int(self.get_parameter('jpeg_quality').value)
        if (not self.device or not 16 <= self.width <= 1920
                or not 16 <= self.height <= 1080 or not math.isfinite(self.fps)
                or not 1 <= self.fps <= 60 or not 1 <= self.quality <= 100):
            raise ValueError('Invalid camera device, dimensions, fps or JPEG quality')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.raw_pub = self.create_publisher(Image, 'camera/image_raw', qos)
        self.jpeg_pub = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos)
        self.bridge = CvBridge()
        self.lock, self.stopping = threading.Lock(), threading.Event()
        self.latest, self.error = None, ''
        self.sequence = self.published = 0
        self.last_warning = 0.0
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
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Driver-dependent best effort.
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
                while not self.stopping.is_set():
                    started = time.monotonic()
                    ok, frame = cap.read()
                    received = time.monotonic()
                    if self.stopping.is_set():
                        break
                    if (not ok or frame is None or not frame.size
                            or received - started > max(.5, 2 / self.fps)):
                        raise RuntimeError('Camera read failed or stalled; reconnecting')
                    # This is host receipt time, not the hardware exposure time.
                    stamp = self.get_clock().now().to_msg()
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    elif frame.ndim == 3 and frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    if frame.ndim != 3 or frame.shape[2] != 3 or str(frame.dtype) != 'uint8':
                        raise RuntimeError('V4L2 did not return an 8-bit BGR image')
                    with self.lock:
                        self.sequence += 1
                        self.latest = self.sequence, frame, stamp, received
                        self.error = ''
            except (cv2.error, RuntimeError, ValueError) as error:
                with self.lock:
                    self.latest, self.error = None, str(error)
            finally:
                cap.release()
            self.stopping.wait(1.0)

    def publish_latest(self):
        with self.lock:
            item, error = self.latest, self.error
        now = time.monotonic()
        if error and now - self.last_warning > 5:
            self.get_logger().warning(error)
            self.last_warning = now
        if item is None or item[0] == self.published or now - item[3] > .5:
            return
        sequence, frame, stamp, _ = item
        self.published = sequence
        try:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw.header.stamp, raw.header.frame_id = stamp, 'camera_optical_frame'
            compressed = CompressedImage()
            compressed.header, compressed.format = raw.header, 'bgr8; jpeg compressed bgr8'
            compressed.data = jpeg.tobytes()
            self.raw_pub.publish(raw)
            self.jpeg_pub.publish(compressed)
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
