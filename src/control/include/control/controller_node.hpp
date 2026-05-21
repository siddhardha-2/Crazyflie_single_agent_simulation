#ifndef CONTROL__CONTROLLER_NODE_HPP_
#define CONTROL__CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "navigation/msg/drone_state.hpp"
#include "control/pid.hpp"

namespace control
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  // -----------------------------------------------------------------------
  // Callbacks
  // -----------------------------------------------------------------------
  void stateCallback(const navigation::msg::DroneState::SharedPtr msg);
  void setpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // -----------------------------------------------------------------------
  // Control loop (timer driven)
  // -----------------------------------------------------------------------
  void controlLoop();

  // -----------------------------------------------------------------------
  // Helpers
  // -----------------------------------------------------------------------
  void loadParameters();
  void enableDrone(bool enable);
  double normalizeAngle(double angle);

  // -----------------------------------------------------------------------
  // Subscribers & Publishers
  // -----------------------------------------------------------------------
  rclcpp::Subscription<navigation::msg::DroneState>::SharedPtr   state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          enable_pub_;

  // -----------------------------------------------------------------------
  // Timer
  // -----------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr control_timer_;

  // -----------------------------------------------------------------------
  // PID controllers (one per axis)
  // -----------------------------------------------------------------------
  std::unique_ptr<PID> pid_x_;
  std::unique_ptr<PID> pid_y_;
  std::unique_ptr<PID> pid_z_;
  std::unique_ptr<PID> pid_yaw_;

  // -----------------------------------------------------------------------
  // Internal state
  // -----------------------------------------------------------------------
  navigation::msg::DroneState::SharedPtr  current_state_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_setpoint_;

  rclcpp::Time last_time_;
  bool         drone_enabled_;
  bool         has_state_;
  bool         has_setpoint_;

  // -----------------------------------------------------------------------
  // Parameters
  // -----------------------------------------------------------------------
  double max_vel_xy_;
  double max_vel_z_;
  double max_yaw_rate_;
  double control_rate_;
};

}  // namespace control

#endif  // CONTROL__CONTROLLER_NODE_HPP_