"""ROS-independent timestamp matching used by RF2O recording and shadow mode."""


def scan_ready(scan_ns, received_ns, now_ns, odom_history, wait_s):
    """RF2O publishes after its input scan; wait for that stamp before sampling."""
    return bool(odom_history and odom_history[-1][0] >= scan_ns) or now_ns - received_ns >= int(wait_s * 1e9)


def synchronized_pair(scan_ns, now_ns, image_history, odom_history, healthy,
                      health_received_ns, limits_s):
    """Select source-stamped observations; None means reject this scan.

    Bool health has no source stamp, so only a recent receipt is claimed. It
    does not turn RF2O into independent wheel/ground-truth odometry.
    """
    if not healthy or not 0 <= now_ns - health_received_ns <= int(limits_s["health_age"] * 1e9):
        return None
    if scan_ns <= 0 or not 0 <= now_ns - scan_ns <= int(limits_s["scan_age"] * 1e9):
        return None
    images = [v for v in image_history if 0 < v[0] <= scan_ns]
    odoms = [v for v in odom_history if 0 < v[0] <= scan_ns]
    if not images or not odoms:
        return None
    rgb, odom = images[-1], odoms[-1]
    if scan_ns - rgb[0] > int(limits_s["image_age"] * 1e9) or scan_ns - odom[0] > int(limits_s["odom_age"] * 1e9):
        return None
    return rgb, odom
