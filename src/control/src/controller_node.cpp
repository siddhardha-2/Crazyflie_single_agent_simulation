#include "control/controller_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace control
{

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
ControllerNode::ControllerNode()
: Node("controller_node"),
  drone_enabled_(false),
  has_state_(false),
  has_setpoint_(false)
{
  loadParameters();

  // Subscribers
  state_sub_ = this->create_subscription<navigation::msg::DroneState>(
    "/cf0/state", 10,
    std::bind(&ControllerNode::stateCallback, this, std::placeholders::_1));

  setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cf0/setpoint", 10,
    std::bind(&ControllerNode::setpointCallback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cf0/cmd_vel", 10);

  enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/cf0/enable", 10);

  // Control timer
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControllerNode::controlLoop, this));

  last_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "ControllerNode started at %.1f Hz", control_rate_);
}

// -----------------------------------------------------------------------
// Load Parameters
// -----------------------------------------------------------------------
void ControllerNode::loadParameters()
{
  this->declare_parameter("pid_x.kp",  1.0);
  this->declare_parameter("pid_x.ki",  0.0);
  this->declare_parameter("pid_x.kd",  0.2);

  this->declare_parameter("pid_y.kp",  1.0);
  this->declare_parameter("pid_y.ki",  0.0);
  this->declare_parameter("pid_y.kd",  0.2);

  this->declare_parameter("pid_z.kp",  1.5);
  this->declare_parameter("pid_z.ki",  0.05);
  this->declare_parameter("pid_z.kd",  0.3);

  this->declare_parameter("pid_yaw.kp", 1.0);
  this->declare_parameter("pid_yaw.ki", 0.0);
  this->declare_parameter("pid_yaw.kd", 0.1);

  this->declare_parameter("max_vel_xy",   0.5);
  this->declare_parameter("max_vel_z",    0.3);
  this->declare_parameter("max_yaw_rate", 0.5);
  this->declare_parameter("control_rate", 50.0);

  // Read values
  double x_kp  = this->get_parameter("pid_x.kp").as_double();
  double x_ki  = this->get_parameter("pid_x.ki").as_double();
  double x_kd  = this->get_parameter("pid_x.kd").as_double();

  double y_kp  = this->get_parameter("pid_y.kp").as_double();
  double y_ki  = this->get_parameter("pid_y.ki").as_double();
  double y_kd  = this->get_parameter("pid_y.kd").as_double();

  double z_kp  = this->get_parameter("pid_z.kp").as_double();
  double z_ki  = this->get_parameter("pid_z.ki").as_double();
  double z_kd  = this->get_parameter("pid_z.kd").as_double();

  double yaw_kp = this->get_parameter("pid_yaw.kp").as_double();
  double yaw_ki = this->get_parameter("pid_yaw.ki").as_double();
  double yaw_kd = this->get_parameter("pid_yaw.kd").as_double();

  max_vel_xy_   = this->get_parameter("max_vel_xy").as_double();
  max_vel_z_    = this->get_parameter("max_vel_z").as_double();
  max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
  control_rate_ = this->get_parameter("control_rate").as_double();

  // Create PID instances
  pid_x_   = std::make_unique<PID>(x_kp,   x_ki,   x_kd,   -max_vel_xy_,   max_vel_xy_);
  pid_y_   = std::make_unique<PID>(y_kp,   y_ki,   y_kd,   -max_vel_xy_,   max_vel_xy_);
  pid_z_   = std::make_unique<PID>(z_kp,   z_ki,   z_kd,   -max_vel_z_,    max_vel_z_);
  pid_yaw_ = std::make_unique<PID>(yaw_kp, yaw_ki, yaw_kd, -max_yaw_rate_, max_yaw_rate_);
}

// -----------------------------------------------------------------------
// State Callback
// -----------------------------------------------------------------------
void ControllerNode::stateCallback(const navigation::msg::DroneState::SharedPtr msg)
{
  current_state_ = msg;
  has_state_ = true;
}

// -----------------------------------------------------------------------
// Setpoint Callback
// -----------------------------------------------------------------------
void ControllerNode::setpointCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_setpoint_ = msg;

  // Reset PIDs on new setpoint to clear windup
  if (!has_setpoint_) {
    pid_x_->reset();
    pid_y_->reset();
    pid_z_->reset();
    pid_yaw_->reset();
  }

  has_setpoint_ = true;

  // Enable drone on first setpoint
  if (!drone_enabled_) {
    enableDrone(true);
  }
}

// -----------------------------------------------------------------------
// Control Loop
// -----------------------------------------------------------------------
void ControllerNode::controlLoop()
{
  if (!has_state_ || !has_setpoint_) {
    return;
  }

  // Compute dt
  rclcpp::Time now = this->get_clock()->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // Desired position from setpoint
  double des_x   = current_setpoint_->pose.position.x;
  double des_y   = current_setpoint_->pose.position.y;
  double des_z   = current_setpoint_->pose.position.z;

  // Desired yaw from setpoint quaternion
  tf2::Quaternion q(
    current_setpoint_->pose.orientation.x,
    current_setpoint_->pose.orientation.y,
    current_setpoint_->pose.orientation.z,
    current_setpoint_->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double des_roll, des_pitch, des_yaw;
  m.getRPY(des_roll, des_pitch, des_yaw);

  // Current state
  double cur_x   = current_state_->x;
  double cur_y   = current_state_->y;
  double cur_z   = current_state_->z;
  double cur_yaw = current_state_->yaw;

  // Compute yaw normalized error
  double yaw_error = normalizeAngle(des_yaw - cur_yaw);

  // Run PIDs
  double vx  = pid_x_->compute(des_x, cur_x, dt);
  double vy  = pid_y_->compute(des_y, cur_y, dt);
  double vz  = pid_z_->compute(des_z, cur_z, dt);
  double wz  = pid_yaw_->compute(des_yaw, cur_yaw + yaw_error, dt);

  // Publish cmd_vel
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = vx;
  cmd.linear.y  = vy;
  cmd.linear.z  = vz;
  cmd.angular.z = wz;

  cmd_vel_pub_->publish(cmd);
}

// -----------------------------------------------------------------------
// Enable Drone
// -----------------------------------------------------------------------
void ControllerNode::enableDrone(bool enable)
{
  std_msgs::msg::Bool msg;
  msg.data = enable;
  enable_pub_->publish(msg);
  drone_enabled_ = enable;
  RCLCPP_INFO(this->get_logger(), "Drone %s", enable ? "ENABLED" : "DISABLED");
}
// -----------------------------------------------------------------------
// Normalize Angle to [-pi, pi]
// -----------------------------------------------------------------------
double ControllerNode::normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace control

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
