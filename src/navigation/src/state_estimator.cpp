#include "navigation/state_estimator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace navigation
{

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
StateEstimator::StateEstimator()
: Node("state_estimator")
{
  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/cf0/odom", 10,
    std::bind(&StateEstimator::odomCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/cf0/imu", 10,
    std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));

  // Publisher
  state_pub_ = this->create_publisher<navigation::msg::DroneState>(
    "/cf0/state", 10);

  RCLCPP_INFO(this->get_logger(), "StateEstimator node started");
}

// -----------------------------------------------------------------------
// Odom Callback
// -----------------------------------------------------------------------
void StateEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
  publishState();
}

// -----------------------------------------------------------------------
// IMU Callback
// -----------------------------------------------------------------------
void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_imu_ = msg;
}

// -----------------------------------------------------------------------
// Publish State
// -----------------------------------------------------------------------
void StateEstimator::publishState()
{
  // Wait until we have at least odom data
  if (!latest_odom_) {
    return;
  }

  navigation::msg::DroneState state;

  // Header
  state.header.stamp = this->get_clock()->now();
  state.header.frame_id = "world";

  // Position from odom
  state.x = latest_odom_->pose.pose.position.x;
  state.y = latest_odom_->pose.pose.position.y;
  state.z = latest_odom_->pose.pose.position.z;

  // Velocity from odom
  state.vx = latest_odom_->twist.twist.linear.x;
  state.vy = latest_odom_->twist.twist.linear.y;
  state.vz = latest_odom_->twist.twist.linear.z;

  // Orientation from odom
  state.orientation = latest_odom_->pose.pose.orientation;

  // Roll Pitch Yaw from quaternion
  quaternionToRPY(
    state.orientation.x,
    state.orientation.y,
    state.orientation.z,
    state.orientation.w,
    state.roll,
    state.pitch,
    state.yaw);

  // IMU data (if available)
  if (latest_imu_) {
    state.angular_velocity   = latest_imu_->angular_velocity;
    state.linear_acceleration = latest_imu_->linear_acceleration;
  }

  state_pub_->publish(state);
}

// -----------------------------------------------------------------------
// Quaternion to Roll Pitch Yaw
// -----------------------------------------------------------------------
void StateEstimator::quaternionToRPY(
  double qx, double qy, double qz, double qw,
  double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

}  // namespace navigation

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::StateEstimator>());
  rclcpp::shutdown();
  return 0;
}