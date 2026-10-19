"""Timestamped camera + planar LiDAR perception; no Twist or Nav2-goal publishers."""
from collections import Counter,deque
import hashlib
import json
from pathlib import Path
import threading
import time
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from std_msgs.msg import String,Bool
from tf2_ros import Buffer,TransformListener,TransformException
from ament_index_python.packages import get_package_share_directory
from room_semantics.calibration import load_calibration,PinholeCamera,transform_matrix
from room_semantics.detector import GroundingDinoDetector,DEFAULT_REVISION
from room_semantics.geometry import FusionConfig,ground_detection,freshness_reason,reject_conflicting_associations
from room_semantics.tracking import ObjectTracker


def stamp_ns(stamp): return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def tf_matrix(transform):
    t=transform.translation; q=transform.rotation
    return transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])


def decode_rgb(msg):
    channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4,"mono8":1}.get(msg.encoding)
    if channels is None or msg.width<1 or msg.height<1 or msg.step<msg.width*channels:
        raise ValueError("Use decoded raw Image rgb8/bgr8/rgba8/bgra8/mono8")
    raw=np.frombuffer(bytes(msg.data),dtype=np.uint8)
    if len(raw)!=msg.height*msg.step: raise ValueError("Image data/step mismatch")
    pixels=raw.reshape(msg.height,msg.step)[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
    if channels==1: pixels=np.repeat(pixels,3,axis=2)
    else:
        pixels=pixels[:,:,:3]
        if msg.encoding.startswith("bgr"): pixels=pixels[:,:,::-1]
    return PILImage.fromarray(pixels.copy(),"RGB")


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("room_perception")
        default_calibration=str(Path(get_package_share_directory("room_perception"))/"config"/"calibration.yaml")
        defaults={"image_topic":"/camera/image_raw","camera_info_topic":"/camera/camera_info","scan_topic":"/scan",
                  "odom_health_topic":"/room_base/odom_healthy","require_odom_healthy":True,
                  "calibration_file":default_calibration,"image_is_rectified":False,
                  "camera_frame":"camera_optical_frame","map_frame":"map","fixed_frame":"odom",
                  "model_id":"IDEA-Research/grounding-dino-tiny","model_revision":DEFAULT_REVISION,"local_files_only":False,
                  "device":"auto","labels":["person","chair","table","door"],"score_threshold":.35,"text_threshold":.25,
                  "max_image_age_s":.5,"max_scan_age_s":.3,"max_sync_s":.12,"max_health_age_s":.3,
                  "min_cluster_points":3,"tracker_ttl_s":.8,"publish_hz":5.}
        for key,value in defaults.items(): self.declare_parameter(key,value)
        self.p={key:self.get_parameter(key).value for key in defaults}
        self.calibration=load_calibration(self.p["calibration_file"])
        self.calibration_id=hashlib.sha256(Path(self.p["calibration_file"]).read_bytes()).hexdigest()
        c=self.calibration
        if (self.p["camera_frame"],self.p["map_frame"],self.p["fixed_frame"])!=(c.camera_frame,c.map_frame,c.fixed_frame):
            raise ValueError("Node frame parameters and calibration frames differ")
        self.config=FusionConfig(min_points=int(self.p["min_cluster_points"]),max_sync_s=float(self.p["max_sync_s"]),
                                 max_image_age_s=float(self.p["max_image_age_s"]),max_scan_age_s=float(self.p["max_scan_age_s"]),min_score=float(self.p["score_threshold"]))
        if not 0<self.p["publish_hz"]<=20 or not 0<self.p["max_health_age_s"]<=.5:
            raise ValueError("Invalid publish/health limits")
        self.camera=None; self.camera_info_ns=0; self.scans=deque(maxlen=40); self.last_image_ns=0
        self.healthy=False; self.health_received_ns=0; self.rejections=Counter(); self.status="detector_loading"
        self.tracker=ObjectTracker(ttl_s=float(self.p["tracker_ttl_s"]))
        self.tf=Buffer(); self.tf_listener=TransformListener(self.tf,self)
        self.box_pub=self.create_publisher(String,"/room_semantics/detections",10)
        self.object_pub=self.create_publisher(String,"/room_semantics/objects",10)
        self.create_subscription(Image,self.p["image_topic"],self.image_cb,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,self.p["camera_info_topic"],self.info_cb,qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.p["scan_topic"],self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Bool,self.p["odom_health_topic"],self.health_cb,10)
        self.lock=threading.Lock(); self.wakeup=threading.Event(); self.stopping=threading.Event()
        self.pending_image=None; self.completed=None; self.latest_result=None; self.last_pair=None; self.detector_error=None
        self.worker=threading.Thread(target=self.detect_loop,name="grounding_dino_latest_frame",daemon=True); self.worker.start()
        self.create_timer(1./float(self.p["publish_hz"]),self.tick)
        if not c.verified:
            self.get_logger().warning("Calibration is unverified: publishing 2D detections only, metric objects disabled")

    def ros_time(self,ns): return Time(nanoseconds=int(ns),clock_type=self.get_clock().clock_type)

    def image_cb(self,msg):
        ns=stamp_ns(msg.header.stamp); now=self.get_clock().now().nanoseconds
        if ns<=0 or ns>now or ns<=self.last_image_ns: return
        self.last_image_ns=ns
        if msg.header.frame_id!=self.calibration.camera_frame: self.rejections["image_frame_mismatch"]+=1; return
        # A single replaceable slot prevents slow inference from queuing old video.
        with self.lock: self.pending_image=(msg,now,self.camera,self.camera_info_ns)
        self.wakeup.set()

    def info_cb(self,msg):
        try:
            if msg.header.frame_id!=self.calibration.camera_frame: raise ValueError("CameraInfo optical frame mismatch")
            self.camera=PinholeCamera.from_projection(msg.width,msg.height,msg.p,msg.header.frame_id)
            self.camera_info_ns=stamp_ns(msg.header.stamp)
        except (ValueError,TypeError) as exc:
            self.camera=None; self.get_logger().warning(str(exc)); self.rejections["invalid_camera_info"]+=1

    def scan_cb(self,msg):
        ns=stamp_ns(msg.header.stamp)
        if ns<=0 or msg.header.frame_id!=self.calibration.lidar_frame: self.rejections["scan_frame_or_stamp"]+=1; return
        if self.scans and ns<=stamp_ns(self.scans[-1].header.stamp): return
        self.scans.append(msg)

    def health_cb(self,msg):
        self.healthy=bool(msg.data); self.health_received_ns=self.get_clock().now().nanoseconds
        if not self.healthy and self.p["require_odom_healthy"]:
            self.tracker.tracks.clear(); self.last_pair=None

    def detect_loop(self):
        try:
            detector=GroundingDinoDetector(self.p["model_id"],self.p["device"],self.p["score_threshold"],self.p["text_threshold"],
                                          self.p["labels"],self.p["model_revision"],self.p["local_files_only"])
        except Exception as exc:
            self.detector_error=f"{type(exc).__name__}: {exc}"; return
        while not self.stopping.is_set():
            self.wakeup.wait(.2); self.wakeup.clear()
            with self.lock: pending=self.pending_image; self.pending_image=None
            if pending is None: continue
            msg,received,camera,camera_info_ns=pending
            try:
                now=self.get_clock().now().nanoseconds; source=stamp_ns(msg.header.stamp)
                if not 0<=now-source<=int(self.config.max_image_age_s*1e9): continue
                image=decode_rgb(msg); started=time.perf_counter(); boxes=detector.detect(image)
                if self.stopping.is_set(): return
                finished=self.get_clock().now().nanoseconds
                self.detector_error=None
                result={"schema_version":1,"stamp_ns":source,"image_stamp_ns":source,
                        "image_received_ns":int(received),"inference_finished_ns":int(finished),
                        "inference_seconds":time.perf_counter()-started,"frame_id":msg.header.frame_id,
                        "width":msg.width,"height":msg.height,"detections":boxes,
                        "camera_info_stamp_ns":camera_info_ns,
                        "pinhole":dict(width=camera.width,height=camera.height,fx=camera.fx,fy=camera.fy,cx=camera.cx,cy=camera.cy,frame_id=camera.frame_id) if camera is not None else None,
                        "model_id":detector.model_id,"model_revision":detector.resolved_revision or detector.revision,
                        "metric_localization":False}
                with self.lock: self.completed=result
            except Exception as exc:
                self.detector_error=f"{type(exc).__name__}: {exc}"

    def fuse(self,result,now):
        if not self.calibration.verified or not self.p["image_is_rectified"] or not self.calibration.image_is_rectified:
            self.status="calibration_or_rectification_unverified"; self.tracker.tracks.clear(); return
        camera=PinholeCamera(**result["pinhole"]) if result.get("pinhole") else None
        if camera is None or (camera.width,camera.height)!=(result["width"],result["height"]):
            self.status="missing_or_mismatched_camera_info"; return
        if not self.scans: self.status="waiting_for_scan"; return
        image_ns=result["image_stamp_ns"]
        eligible=[s for s in self.scans if abs(stamp_ns(s.header.stamp)-image_ns)<=int(self.config.max_sync_s*1e9)]
        if not eligible: self.status="unsynchronized_scan_image"; return
        # A completed bbox may be reused for a newer scan only within the same
        # tight acquisition-time bound. Its original camera timestamp remains.
        scan=max(eligible,key=lambda s:stamp_ns(s.header.stamp))
        scan_ns=stamp_ns(scan.header.stamp)
        reason=freshness_reason(image_ns,scan_ns,now,self.config)
        if reason: self.status=reason; return
        key=(image_ns,scan_ns)
        if key==self.last_pair: return
        try:
            same_time=self.tf.lookup_transform(self.calibration.camera_frame,self.calibration.lidar_frame,self.ros_time(scan_ns),timeout=Duration(seconds=0.)).transform
            self.calibration.verify_tf(tf_matrix(same_time))
            # Transform laser@scan into optical@image via local odom, preserving
            # both acquisition times. Never use the most recent robot pose.
            optical=self.tf.lookup_transform_full(self.calibration.camera_frame,self.ros_time(image_ns),
                         self.calibration.lidar_frame,self.ros_time(scan_ns),self.calibration.fixed_frame,timeout=Duration(seconds=0.)).transform
            map_tf=self.tf.lookup_transform(self.calibration.map_frame,self.calibration.lidar_frame,self.ros_time(scan_ns),timeout=Duration(seconds=0.)).transform
        except (TransformException,ValueError) as exc:
            self.status="tf_or_calibration_unavailable"; return
        observations=[]
        for detection in result["detections"]:
            try:
                observation,reason=ground_detection(detection,ranges=scan.ranges,angle_min=scan.angle_min,angle_increment=scan.angle_increment,
                             range_min=scan.range_min,range_max=scan.range_max,camera=camera,camera_from_lidar=tf_matrix(optical),map_from_lidar=tf_matrix(map_tf),
                             image_stamp_ns=image_ns,scan_stamp_ns=scan_ns,now_ns=now,config=self.config)
            except (ValueError,TypeError):
                self.rejections["invalid_scan_or_detection_geometry"]+=1; self.tracker.tracks.clear()
                self.status="invalid_scan_or_detection_geometry"; return
            if reason: self.rejections[reason]+=1; continue
            observation.update(image_received_ns=result["image_received_ns"],inference_finished_ns=result["inference_finished_ns"],inference_seconds=result["inference_seconds"],camera_info_stamp_ns=result["camera_info_stamp_ns"])
            observations.append(observation)
        observations,conflicts=reject_conflicting_associations(observations)
        self.rejections["ambiguous_semantic_overlap"]+=conflicts
        self.tracker.update(observations,now); self.last_pair=key
        self.status="ok" if observations else "detections_without_metric_support"

    def tick(self):
        now=self.get_clock().now().nanoseconds
        with self.lock: completed=self.completed; self.completed=None
        if completed is not None:
            completed["published_ns"]=now
            completed["stale"]=not 0<=now-completed["image_stamp_ns"]<=int(self.config.max_image_age_s*1e9)
            msg=String(); msg.data=json.dumps(completed,sort_keys=True,allow_nan=False); self.box_pub.publish(msg)
            self.latest_result=completed
        health_ok=self.healthy and 0<=now-self.health_received_ns<=int(self.p["max_health_age_s"]*1e9)
        if self.p["require_odom_healthy"] and not health_ok:
            self.status="odometry_not_healthy"; self.tracker.tracks.clear()
        elif self.latest_result is not None:
            self.fuse(self.latest_result,now)
        if self.detector_error: self.status="detector_error"
        scene=self.tracker.scene(now,self.calibration.map_frame,self.status,self.rejections)
        scene["perception_status"]=self.status
        scene["calibration_verified"]=self.calibration.verified
        scene["calibration_sha256"]=self.calibration_id
        if self.detector_error: scene["detector_error"]=self.detector_error
        msg=String(); msg.data=json.dumps(scene,sort_keys=True,allow_nan=False); self.object_pub.publish(msg)

    def stop_worker(self):
        self.stopping.set(); self.wakeup.set(); self.worker.join(timeout=2.)


def main(args=None):
    rclpy.init(args=args); node=None
    try:
        node=PerceptionNode(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node is not None: node.stop_worker(); node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__": main()
