// Test the actual bundled RF2O numerical implementation, with ROS-free message/logger stubs.
#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

sensor_msgs::msg::LaserScan room_scan(double robot_x, int frame)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp.sec = 1;
  scan.header.stamp.nanosec = frame * 100000000;
  const int count = 720;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = (scan.angle_max-scan.angle_min)/(count-1);
  scan.range_min = 0.05;
  scan.range_max = 12.0;
  for (int i=0;i<count;++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    const double c = std::cos(angle), s = std::sin(angle);
    const double rx = c > 0 ? (3.0-robot_x)/c : (-3.0-robot_x)/c;
    const double ry = s > 0 ? 2.0/s : -2.0/s;
    scan.ranges.push_back(std::min(rx, ry));
  }
  return scan;
}

int main()
{
  geometry_msgs::msg::Pose initial;
  initial.orientation.w = 1.0;
  rf2o::CLaserOdometry2D odom;
  auto first = room_scan(0.0, 0);
  odom.init(first, initial);
  for (int frame=1;frame<=3;++frame) {
    auto scan = room_scan(0.0, frame);
    odom.current_scan_time = scan.header.stamp;
    if (!odom.odometryCalculation(scan)) {
      std::cerr << "Observable stationary room was rejected at frame " << frame << "\n";
      return 1;
    }
    assert(odom.robot_pose_.matrix().allFinite());
    assert(std::abs(odom.robot_pose_.translation()(0)) < 1e-4);
    assert(std::abs(odom.robot_pose_.translation()(1)) < 1e-4);
    assert(std::abs(odom.lin_speed) < 1e-4);
  }
  auto moved = room_scan(0.01, 4);
  odom.current_scan_time = moved.header.stamp;
  if (!odom.odometryCalculation(moved)) return 2;
  if (!(odom.robot_pose_.translation()(0) > 0.003 && odom.robot_pose_.translation()(0) < 0.025)) {
    std::cerr << "Wrong translation " << odom.robot_pose_.translation()(0) << "\n";
    return 3;
  }
  // Real algorithm must reject empty returns; it must not report a fresh identity estimate.
  rf2o::CLaserOdometry2D empty;
  auto blank = room_scan(0.0, 0);
  std::fill(blank.ranges.begin(), blank.ranges.end(), 0.f);
  empty.init(blank, initial);
  blank.header.stamp.nanosec = 100000000;
  empty.current_scan_time = blank.header.stamp;
  assert(!empty.odometryCalculation(blank));
  // A perfectly circular scene cannot observe yaw: normal equations must reject it.
  rf2o::CLaserOdometry2D circle;
  auto round = room_scan(0.0, 0);
  std::fill(round.ranges.begin(), round.ranges.end(), 2.f);
  circle.init(round, initial);
  round.header.stamp.nanosec = 100000000;
  circle.current_scan_time = round.header.stamp;
  assert(!circle.odometryCalculation(round));
  std::cout << "RF2O math host checks passed: stationary room, translation, empty/rank-deficient rejection\n";
}
