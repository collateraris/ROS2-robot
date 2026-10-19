#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace builtin_interfaces { namespace msg { struct Time { int32_t sec=0; uint32_t nanosec=0; }; }}
namespace std_msgs { namespace msg { struct Header { builtin_interfaces::msg::Time stamp; std::string frame_id; }; }}
namespace geometry_msgs { namespace msg {
struct Point { double x=0, y=0, z=0; };
struct Quaternion { double x=0, y=0, z=0, w=0; };
struct Pose { Point position; Quaternion orientation; };
}}
namespace nav_msgs { namespace msg { struct Odometry {}; }}
namespace sensor_msgs { namespace msg { struct LaserScan {
  std_msgs::msg::Header header;
  float angle_min=0, angle_max=0, angle_increment=0, range_min=0, range_max=0;
  std::vector<float> ranges;
}; }}
