"""Strict symbolic plans, current-scene grounding, geometry and cancellation tokens.

The model never supplies coordinates. Metric navigation goals are derived only
from trusted, source-stamped camera/LiDAR detections or confirmed waypoints.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re


class PlanRejected(ValueError):
    pass


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PlanRejected(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def strict_json(text, maximum=131072):
    if not isinstance(text, str) or len(text.encode("utf-8")) > maximum:
        raise PlanRejected("JSON too large or not text")
    try:
        return json.loads(text, object_pairs_hook=_unique,
                          parse_constant=lambda v: (_ for _ in ()).throw(PlanRejected(f"invalid JSON number: {v}")))
    except (json.JSONDecodeError, RecursionError) as exc:
        raise PlanRejected("invalid JSON") from exc


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def normalize(text):
    return " ".join(str(text).casefold().replace("ё", "е").strip().split()).strip(".! ")


STOP_WORDS = {"стоп", "остановись", "stop", "отмена", "cancel"}


def is_stop(text):
    return normalize(text) in STOP_WORDS


def parse_command(text):
    """ROS command supports plain text or a correlated evaluator request."""
    if not isinstance(text, str) or not text.strip() or len(text) > 2048:
        raise PlanRejected("instruction must contain 1..2048 characters")
    if text.lstrip().startswith("{"):
        request = strict_json(text, 8192)
        if not isinstance(request, dict) or set(request) != {"request_id", "instruction"}:
            raise PlanRejected("command JSON needs only request_id and instruction")
        if not isinstance(request["request_id"], str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,100}", request["request_id"]):
            raise PlanRejected("invalid request_id")
        if not isinstance(request["instruction"], str) or not request["instruction"].strip() or len(request["instruction"]) > 2048:
            raise PlanRejected("invalid instruction")
        return request["request_id"], request["instruction"]
    return "", text


@dataclass(frozen=True)
class SceneObject:
    id: str
    label: str
    x: float
    y: float
    z: float
    uncertainty_m: float
    last_seen_ns: int
    score: float


@dataclass(frozen=True)
class Scene:
    stamp_ns: int
    image_stamp_ns: int
    scan_stamp_ns: int
    objects: tuple[SceneObject, ...]

    def lookup(self, identifier):
        found = [obj for obj in self.objects if obj.id == identifier]
        if len(found) != 1:
            raise PlanRejected(f"unknown or lost object: {identifier}")
        return found[0]

    def prompt_data(self):
        return [{"id": obj.id, "label": obj.label, "score": obj.score}
                for obj in self.objects]


def parse_scene(text):
    doc = strict_json(text)
    if not isinstance(doc, dict) or doc.get("schema_version") != 1 or isinstance(doc.get("schema_version"), bool):
        raise PlanRejected("scene schema_version must be 1")
    if doc.get("frame_id") != "map":
        raise PlanRejected("scene needs metric map frame")
    for key in ("stamp_ns", "image_stamp_ns", "scan_stamp_ns"):
        if type(doc.get(key)) is not int or doc[key] <= 0:
            raise PlanRejected(f"invalid scene {key}")
    rows = doc.get("objects")
    if not isinstance(rows, list) or len(rows) > 100:
        raise PlanRejected("scene objects must be a bounded list")
    seen, objects = set(), []
    for row in rows:
        if not isinstance(row, dict):
            raise PlanRejected("invalid scene object")
        identifier, label = row.get("id"), row.get("label")
        if not isinstance(identifier, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", identifier) or identifier in seen:
            raise PlanRejected("invalid or duplicate object ID")
        if not isinstance(label, str) or not re.fullmatch(r"[a-z][a-z0-9_ ]{0,49}", label):
            raise PlanRejected("invalid canonical object label")
        position = row.get("position")
        if not isinstance(position, dict) or set(position) != {"x", "y", "z"} or not all(finite(v) for v in position.values()):
            raise PlanRejected("invalid metric object position")
        if row.get("source") != "camera_lidar":
            raise PlanRejected("object needs trusted camera_lidar source")
        score, uncertainty, stamp = row.get("score"), row.get("uncertainty_m"), row.get("last_seen_ns")
        if not finite(score) or not 0 <= score <= 1 or not finite(uncertainty) or not 0 <= uncertainty <= 10:
            raise PlanRejected("invalid object score/uncertainty")
        if type(stamp) is not int or stamp <= 0:
            raise PlanRejected("invalid object acquisition stamp")
        seen.add(identifier)
        objects.append(SceneObject(identifier, label, float(position["x"]), float(position["y"]),
                                   float(position["z"]), float(uncertainty), stamp, float(score)))
    return Scene(doc["stamp_ns"], doc["image_stamp_ns"], doc["scan_stamp_ns"], tuple(objects))


@dataclass(frozen=True)
class Step:
    action: str
    object_id: str = ""
    name: str = ""
    seconds: float = 0.0

    def document(self):
        value = {"action": self.action}
        if self.action == "goto_object": value["object_id"] = self.object_id
        if self.action == "goto_waypoint": value["name"] = self.name
        if self.action == "wait": value["seconds"] = self.seconds
        return value


@dataclass(frozen=True)
class Plan:
    steps: tuple[Step, ...]

    def document(self):
        return {"steps": [step.document() for step in self.steps]}


def parse_plan(text, object_ids, waypoints):
    doc = strict_json(text)
    if not isinstance(doc, dict) or set(doc) != {"steps"}:
        raise PlanRejected("plan must contain only steps; numeric goals are forbidden")
    rows = doc["steps"]
    if not isinstance(rows, list) or not 1 <= len(rows) <= 8:
        raise PlanRejected("plan needs 1..8 steps")
    keys = {"goto_object": {"action", "object_id"}, "goto_waypoint": {"action", "name"},
            "wait": {"action", "seconds"}, "stop": {"action"}}
    result, wait_total = [], 0.0
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or not isinstance(row.get("action"), str) or row["action"] not in keys:
            raise PlanRejected("unknown plan action")
        action = row["action"]
        if set(row) != keys[action]:
            raise PlanRejected(f"extra/missing fields for {action}")
        if action == "goto_object":
            if not isinstance(row["object_id"], str) or row["object_id"] not in object_ids:
                raise PlanRejected("model referenced an unknown object ID")
            result.append(Step(action, object_id=row["object_id"]))
        elif action == "goto_waypoint":
            if not isinstance(row["name"], str) or row["name"] not in waypoints:
                raise PlanRejected("model referenced an unknown/disabled waypoint")
            result.append(Step(action, name=row["name"]))
        elif action == "wait":
            if not finite(row["seconds"]) or not 0 <= row["seconds"] <= 10:
                raise PlanRejected("wait seconds must be in [0,10]")
            wait_total += row["seconds"]
            if wait_total > 30: raise PlanRejected("total wait time exceeds 30 seconds")
            result.append(Step(action, seconds=float(row["seconds"])))
        else:
            if i != len(rows)-1: raise PlanRejected("stop must be the final step")
            result.append(Step(action))
    return Plan(tuple(result))


def explicit_id(instruction, identifier):
    return re.search(r"(?<![A-Za-z0-9_-])" + re.escape(identifier.casefold()) + r"(?![A-Za-z0-9_-])",
                     instruction.casefold()) is not None


def validate_grounding(plan, instruction, scene, now_ns, max_age_s=1.0,
                       max_uncertainty_m=.3, minimum_score=.5, reference=None,
                       moved_limit_m=.3):
    for step in plan.steps:
        if step.action != "goto_object": continue
        obj = scene.lookup(step.object_id)
        for stamp in (scene.stamp_ns, scene.image_stamp_ns, scene.scan_stamp_ns, obj.last_seen_ns):
            age = (now_ns - stamp) * 1e-9
            if not -.05 <= age <= max_age_s:
                raise PlanRejected("object/scene acquisition time is stale or in the future")
        if obj.uncertainty_m > max_uncertainty_m or obj.score < minimum_score:
            raise PlanRejected("object metric uncertainty/score does not meet limits")
        same = [o for o in scene.objects if o.label == obj.label and -.05 <= (now_ns-o.last_seen_ns)*1e-9 <= max_age_s]
        if len(same) > 1 and not explicit_id(instruction, obj.id):
            raise PlanRejected(f"ambiguous {obj.label}; name an observed object ID explicitly")
        if reference is not None:
            old = reference.lookup(obj.id)
            if old.label != obj.label or math.hypot(old.x-obj.x, old.y-obj.y) > moved_limit_m:
                raise PlanRejected("object moved/changed during planning or execution; replan explicitly")
    return True


@dataclass(frozen=True)
class MetricGoal:
    x: float
    y: float
    yaw: float


def planar_quaternion(x,y,z,w):
    values=(x,y,z,w)
    if not all(finite(v) for v in values) or abs(sum(v*v for v in values)-1)>.01 or abs(x)>.01 or abs(y)>.01:
        raise PlanRejected("nonplanar or invalid orientation")
    return math.atan2(2*w*z,1-2*z*z)


def validate_path(points,frame,robot,goal):
    """points: (frame,x,y,z,qx,qy,qz,qw), from the Nav2 path result."""
    if frame!="map" or not 2<=len(points)<=20000:
        raise PlanRejected("empty/invalid map path")
    for point in points:
        if len(point)!=8:
            raise PlanRejected("invalid path pose")
        local_frame,x,y,z,qx,qy,qz,qw=point
        if local_frame not in ("","map") or not all(finite(v) for v in (x,y,z)) or abs(z)>.02:
            raise PlanRejected("invalid path coordinate/frame")
        planar_quaternion(qx,qy,qz,qw)
    first,last=points[0],points[-1]
    if math.hypot(first[1]-robot.x,first[2]-robot.y)>.5 or math.hypot(last[1]-goal.x,last[2]-goal.y)>.2:
        raise PlanRejected("path endpoints do not match current robot and grounded goal")
    return True


def standoff_goal(robot_x, robot_y, obj, standoff_m=.8):
    if not all(finite(v) for v in (robot_x, robot_y, standoff_m)) or not .3 <= standoff_m <= 3.:
        raise PlanRejected("invalid standoff geometry")
    dx, dy = obj.x-robot_x, obj.y-robot_y
    distance = math.hypot(dx, dy)
    if distance <= standoff_m:
        return None
    yaw = math.atan2(dy, dx)
    return MetricGoal(obj.x-standoff_m*math.cos(yaw), obj.y-standoff_m*math.sin(yaw), yaw)


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]

    def __post_init__(self):
        if type(self.width) is not int or type(self.height) is not int or min(self.width,self.height) < 1 or self.width*self.height > 20_000_000:
            raise PlanRejected("invalid map dimensions")
        if not finite(self.resolution) or self.resolution <= 0 or not all(finite(v) for v in (self.origin_x,self.origin_y,self.origin_yaw)):
            raise PlanRejected("invalid map geometry")
        if len(self.data) != self.width*self.height or any(type(v) is not int or v < -1 or v > 100 for v in self.data):
            raise PlanRejected("invalid occupancy data")

    def local(self, x, y):
        dx,dy=x-self.origin_x,y-self.origin_y
        c,s=math.cos(self.origin_yaw),math.sin(self.origin_yaw)
        return c*dx+s*dy,-s*dx+c*dy

    def check(self, goal, clearance_m, obstacle_threshold=65):
        if not isinstance(goal,MetricGoal) or not all(finite(v) for v in (goal.x,goal.y,goal.yaw,clearance_m)) or clearance_m <= 0:
            raise PlanRejected("invalid goal/clearance")
        x,y=self.local(goal.x,goal.y); r=clearance_m; res=self.resolution
        if x-r < 0 or y-r < 0 or x+r >= self.width*res or y+r >= self.height*res:
            raise PlanRejected("goal clearance is outside map bounds")
        # Include all grid squares touched by the disk, not only cell centres.
        for gy in range(math.floor((y-r)/res),math.floor((y+r)/res)+1):
            for gx in range(math.floor((x-r)/res),math.floor((x+r)/res)+1):
                closest_x=max(gx*res,min(x,(gx+1)*res))
                closest_y=max(gy*res,min(y,(gy+1)*res))
                if (closest_x-x)**2+(closest_y-y)**2 > r*r: continue
                value=self.data[gy*self.width+gx]
                if value < 0 or value >= obstacle_threshold:
                    raise PlanRejected("goal footprint touches unknown/occupied map cells")
        return True


def validate_grid_path(points,grid,clearance_m,max_samples=100000):
    """Check the entire piecewise-linear path against the latest map.

    Nav2's own collision checking remains necessary. This additional gate checks
    the circular robot footprint along every segment at <= half a map cell,
    including sparse/jumping path poses, with a bounded amount of work.
    """
    if not isinstance(grid,OccupancyGrid) or not 2<=len(points)<=20000:
        raise PlanRejected("invalid occupancy path/map")
    if type(max_samples) is not int or not 1<=max_samples<=100000:
        raise PlanRejected("invalid path sampling bound")
    if not finite(clearance_m) or clearance_m<=0:
        raise PlanRejected("invalid path footprint clearance")
    for point in points:
        if len(point)!=8 or not all(finite(v) for v in point[1:3]):
            raise PlanRejected("invalid path coordinate")
    spacing=grid.resolution*.5
    intervals=[];total=1
    for first,last in zip(points,points[1:]):
        count=max(1,math.ceil(math.hypot(last[1]-first[1],last[2]-first[2])/spacing))
        total+=count
        if total>max_samples:
            raise PlanRejected("occupancy path sampling budget exceeded")
        intervals.append(count)
    grid.check(MetricGoal(points[0][1],points[0][2],0.),clearance_m)
    for first,last,count in zip(points,points[1:],intervals):
        dx,dy=last[1]-first[1],last[2]-first[2]
        for index in range(1,count+1):
            fraction=index/count
            grid.check(MetricGoal(first[1]+fraction*dx,first[2]+fraction*dy,0.),clearance_m)
    return True


class GenerationGate:
    """Pure cancellation contract shared by asynchronous planner/action callbacks."""
    def __init__(self):
        self.generation=0
        self.active=False

    def begin(self):
        self.generation += 1
        self.active=True
        return self.generation

    def cancel(self):
        self.generation += 1
        self.active=False

    def current(self,token):
        return self.active and token == self.generation

    def accept_handle(self,token,handle):
        if not self.current(token):
            if getattr(handle,"accepted",False): handle.cancel_goal_async()
            return False
        return bool(getattr(handle,"accepted",False))
