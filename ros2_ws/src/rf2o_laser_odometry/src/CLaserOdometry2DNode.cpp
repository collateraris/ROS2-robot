/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "/base_pose_ground_truth");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);
  max_scan_age = this->declare_parameter<double>("max_scan_age", 0.25);
  max_scan_gap = this->declare_parameter<double>("max_scan_gap", 0.50);
  min_valid_ratio = this->declare_parameter<double>("min_valid_ratio", 0.20);
  if (!std::isfinite(freq) || freq < 1.0 || freq > 100.0 ||
      !std::isfinite(max_scan_age) || max_scan_age <= 0.0 ||
      !std::isfinite(max_scan_gap) || max_scan_gap <= 0.0 ||
      !std::isfinite(min_valid_ratio) || min_valid_ratio <= 0.0 || min_valid_ratio > 1.0)
    throw std::invalid_argument("Invalid RF2O rate/scan validation parameters");
  new_scan_available = false;
  fault_latched = false;

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  
  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (!GT_pose_initialized || fault_latched) return;
  const auto stamp = rclcpp::Time(new_scan->header.stamp, get_clock()->get_clock_type());
  const double age = (get_clock()->now() - stamp).seconds();
  const size_t count = new_scan->ranges.size();
  const bool geometry_ok = count >= 64 && count <= 20000 &&
      !new_scan->header.frame_id.empty() && std::isfinite(new_scan->angle_min) &&
      std::isfinite(new_scan->angle_max) && std::isfinite(new_scan->angle_increment) &&
      new_scan->angle_increment > 0.0f && new_scan->angle_max > new_scan->angle_min &&
      std::abs(new_scan->angle_min + new_scan->angle_max) < 0.03 &&
      std::abs((count - 1) * new_scan->angle_increment -
               (new_scan->angle_max - new_scan->angle_min)) < 0.03 &&
      std::isfinite(new_scan->range_min) && std::isfinite(new_scan->range_max) &&
      new_scan->range_min > 0.0f && new_scan->range_max > new_scan->range_min;
  sensor_msgs::msg::LaserScan clean = *new_scan;
  size_t valid = 0;
  for (auto &range : clean.ranges) {
    if (std::isfinite(range) && range >= clean.range_min && range <= clean.range_max)
      ++valid;
    else
      range = 0.0f;  // RF2O's missing-return representation, never free space.
  }
  const bool quality_ok = valid >= 64 && valid >= min_valid_ratio * count;
  if (!geometry_ok || !quality_ok || !std::isfinite(age) || age < -0.05 || age > max_scan_age) {
    if (!rf2o_ref.first_laser_scan) fault_latched = true;
    new_scan_available = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Invalid/stale scan: no odometry; restart after a running-session fault");
    return;
  }
  if (!rf2o_ref.first_laser_scan) {
    const auto previous_stamp = rclcpp::Time(last_scan.header.stamp, get_clock()->get_clock_type());
    const double elapsed = (stamp - previous_stamp).seconds();
    const bool same_geometry = count == rf2o_ref.width &&
        clean.header.frame_id == last_scan.header.frame_id &&
        std::abs(clean.angle_min - last_scan.angle_min) < 1e-5 &&
        std::abs(clean.angle_max - last_scan.angle_max) < 1e-5;
    if (!same_geometry || elapsed <= 0.0 || elapsed > max_scan_gap) {
      fault_latched = true;
      new_scan_available = false;
      RCLCPP_ERROR(get_logger(), "RF2O scan geometry/time discontinuity; restart and relocalize");
      return;
    }
    last_scan = std::move(clean);
    rf2o_ref.current_scan_time = rclcpp::Time(last_scan.header.stamp);
    new_scan_available = true;
  } else {
    last_scan = std::move(clean);
    // TF listener may not have received the static transform at startup. Retry safely.
    if (!setLaserPoseFromTf()) return;
    rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
    rf2o_ref.first_laser_scan = false;
  }
}


/** 
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{  
  bool retrieved = false;  
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "%s",ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o 
  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  // Do only run when a new scan is ready 
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    // Reject solver failures. Publishing an unchanged pose with a fresh stamp is unsafe.
    if (!rf2o_ref.odometryCalculation(last_scan) ||
        !rf2o_ref.robot_pose_.matrix().allFinite() ||
        !std::isfinite(rf2o_ref.lin_speed) || !std::isfinite(rf2o_ref.ang_speed)) {
      fault_latched = true;
      new_scan_available = false;
      RCLCPP_ERROR(get_logger(), "RF2O solver rejected scan; restart and relocalize");
      return;
    }

    // The output retains acquisition time, never the timer/processing wall clock.
    publish();

    // Do not run on the same data!
    new_scan_available = false;
  }
  else
  {
    // This is a warning. We depend on laser scans, so no meaning running faster than scan freq.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for valid new laser scan...");
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  
  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  //publish the message
  odom_pub->publish(odom);

  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
  }
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Loop
  while (rclcpp::ok()){ 
      rclcpp::spin_some(myLaserOdomNode);
      myLaserOdomNode->process();
      rate.sleep();
  }

  return 0;
}