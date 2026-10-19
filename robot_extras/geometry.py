"""Pure geometry for a one-shot, stationary-person approach task."""
import math


def standoff_goal(robot, target, distance=1.0, maximum_range=5.0):
    values = (*robot, *target, distance, maximum_range)
    if len(robot) != 2 or len(target) != 2 or not all(math.isfinite(x) for x in values):
        raise ValueError("Finite planar metric positions required")
    if distance <= 0 or maximum_range <= distance:
        raise ValueError("Invalid range or standoff")
    dx, dy = target[0]-robot[0], target[1]-robot[1]
    radius = math.hypot(dx, dy)
    if radius > maximum_range:
        raise ValueError("Target outside configured observation range")
    if radius <= distance:
        return None
    factor = (radius-distance)/radius
    return robot[0]+factor*dx, robot[1]+factor*dy, math.atan2(dy, dx)


def range_clear(value, minimum, maximum, stop_distance, allow_positive_infinity=False):
    if not all(math.isfinite(v) for v in (minimum, maximum, stop_distance)):
        return False
    if not 0 <= minimum < maximum or not minimum < stop_distance < maximum:
        return False
    if value == math.inf:
        return bool(allow_positive_infinity)
    return math.isfinite(value) and minimum <= value <= maximum and value > stop_distance
