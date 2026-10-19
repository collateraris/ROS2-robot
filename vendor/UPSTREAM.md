# Reviewed ROS driver sources

Fetched 2026-10-14, official repositories only:

| Package | Repository | Exact revision | Licence |
|---|---|---|---|
| `rf2o_laser_odometry` | https://github.com/MAPIRlab/rf2o_laser_odometry/tree/ros2 | `b38c68e46387b98845ecbfeb6660292f967a00d3` | GPL v3, preserve upstream COPYING/LICENSE |
| `sllidar_ros2` | https://github.com/Slamtec/sllidar_ros2 | `34300099fadfc772965962dec837bf436706188f` | BSD, preserve upstream LICENSE |

Run `bash scripts/fetch_ros_drivers.sh` from any directory to obtain both exact source trees. It refuses to overwrite existing unmarked packages. In the supplied archive the reviewed source trees may already be present. The C1 driver defaults match official `launch/sllidar_c1_launch.py`: serial, 460800 baud, laser frame, Standard scan mode, angle compensation enabled.

## Local RF2O patch

`patches/rf2o_jazzy_guard.patch` changes the ROS2 wrapper and fixes concrete initialization/numerical problems in the pinned source. It is a project patch, not an upstream release or a claim of certified Jazzy compatibility.

- Use current ROS2 package metadata/rosdep dependencies and Eigen target linking; modern tf2 `.hpp` includes.
- Retry missing startup static TF without converting an uninitialized transform.
- Initialize the pose quaternion and first scan pyramid correctly; use a row range matrix as required by the algorithm.
- Initialize the new-scan flag; respect solver failure instead of publishing old pose with a fresh stamp.
- Require unchanged symmetric planar scan geometry, monotonic acquisition timestamps, sufficient finite in-range returns and bounded source age/gaps. Invalid returns are represented by zero only inside RF2O; original `/scan` remains intact for collision detection.
- Latch running-session input/solver faults: restart RF2O and deliberately reset/relocalize before continued navigation. This does not infer safety from mere scan freshness.
- Prevent division by zero on perfectly static scan derivatives. Reject rank-deficient normal equations and nonfinite solver output.
- Body-frame forward velocity comes from the base transform increment, including laser mounting offsets/yaw.
- RF2O output remains `/odom_rf2o`, retaining the actual scan timestamp and `publish_tf: false`. Only `room_odom_guard` emits accepted `/odom` and `odom -> base_link` TF.
- Keep the internal solver node from consuming the outer node's launch rename (duplicate graph names).

A fresh timestamp does not prove a correct scan match. Glass, moving people, a narrow corridor, weak geometry, vibration, spinning/slipping, dynamic scenes and nonplanar mounting can still degrade laser odometry. The guard rejects observable failures, but is not a full RF2O uncertainty estimator. Conservative speed limits and calibrated PWM remain necessary without encoders.

Upstream issue https://github.com/MAPIRlab/rf2o_laser_odometry/issues/43 reports a Jazzy waiting-for-scans problem. This patch addresses observed source defects; resolving that specific issue and verifying actual ROS2 build, LiDAR TF and sensor transport requires the user's Jazzy robot environment. No ROS runtime was available in the authoring container.

## Checks performed

- Patched source applied cleanly to the exact pinned upstream revision; checked no whitespace errors.
- Eight offline room navigation tests passed, constructing the real launch logic with minimal launch object stubs. They check RF2O/C1 settings, guarded odometry ownership, the velocity chain, no duplicate optional sensor nodes, push mapping, and invalid arguments.
- The actual patched RF2O numerical source was compiled with GCC C++14 and official Eigen 3.4.0 (`3147391d946bb4b6c68edd901f2add6ac1f31f8c`). Only ROS message/logger/time classes were replaced by test stubs. The executable passed stationary rectangular room, 1 cm forward motion, empty returns and rotationally ambiguous circular-scene rejection checks.

Repeat the last check on a system with `libeigen3-dev` installed:

```bash
bash vendor/tests/check_rf2o_math.sh /usr/include/eigen3
```

These are source/math checks, not an executing Jazzy ROS graph or a hardware test. RF2O wrapper compilation, TF timing, serial transport and real scene tracking remain to be validated with the robot.
