#ifndef NAVIGATION__STATE_ESTIMATOR_HPP_
#define NAVIGATION__STATE_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "navigation/msg/drone_state.hpp"

namespace navigation
{

class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator();

private:
  // -----------------------------------------------------------------------
  // Callbacks
  // -----------------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // -----------------------------------------------------------------------
  // Helper
  // -----------------------------------------------------------------------
  void publishState();
  void quaternionToRPY(
    double qx, double qy, double qz, double qw,
    double & roll, double & pitch, double & yaw);

  // -----------------------------------------------------------------------
  // Subscribers & Publisher
  // -----------------------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<navigation::msg::DroneState>::SharedPtr state_pub_;

  // -----------------------------------------------------------------------
  // Internal state (latest data from each topic)
  // -----------------------------------------------------------------------
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::Imu::SharedPtr   latest_imu_;
};

}  // namespace navigation

#endif  // NAVIGATION__STATE_ESTIMATOR_HPP_