#pragma once
#include "messages.hpp"
#include <memory>
namespace rclcpp {
class Duration { double value_; public: explicit Duration(double t):value_(t){} double seconds() const { return value_; } };
class Time {
  double value_;
public:
  Time():value_(0){}
  Time(const builtin_interfaces::msg::Time &t):value_(t.sec+1e-9*t.nanosec){}
  explicit Time(double t):value_(t){}
  Time &operator=(const builtin_interfaces::msg::Time &t) { value_=t.sec+1e-9*t.nanosec; return *this; }
  Duration operator-(const Time &other) const { return Duration(value_-other.value_); }
};
class Clock { public: Time now() const { return Time(0.0); } };
class NodeOptions { public: NodeOptions &use_global_arguments(bool) {return *this;} NodeOptions &enable_rosout(bool) {return *this;} };
class Logger {};
class Node {
public:
  Node(const char *, const NodeOptions & = NodeOptions()){}
  std::shared_ptr<Clock> get_clock() const {return std::make_shared<Clock>();}
  Logger get_logger() const {return Logger();}
};
}
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
