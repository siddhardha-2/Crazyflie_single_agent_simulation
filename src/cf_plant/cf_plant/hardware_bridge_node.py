import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from crazyflie_interfaces.msg import Hover
from crazyflie_interfaces.srv import Arm, Takeoff
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('cf_hardware_bridge')

        self.declare_parameter('robot_name', 'cf0')
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('takeoff_duration', 3.0)
        self.declare_parameter('command_rate', 50.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_takeoff = self.get_parameter('auto_takeoff').value
        self.takeoff_duration = self.get_parameter('takeoff_duration').value
        self.command_rate = self.get_parameter('command_rate').value

        self.pose_topic = f'/{self.robot_name}/pose'
        self.odom_topic = f'/{self.robot_name}/odom'
        self.setpoint_topic = f'/{self.robot_name}/setpoint'
        self.cmd_vel_topic = f'/{self.robot_name}/cmd_vel'
        self.cmd_hover_topic = f'/{self.robot_name}/cmd_hover'
        self.arm_service_name = f'/{self.robot_name}/arm'
        self.takeoff_service_name = f'/{self.robot_name}/takeoff'

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            self.setpoint_topic,
            self.setpoint_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10
        )

        self.cmd_hover_pub = self.create_publisher(
            Hover,
            self.cmd_hover_topic,
            10
        )

        self.arm_client = self.create_client(Arm, self.arm_service_name)
        self.takeoff_client = self.create_client(Takeoff, self.takeoff_service_name)

        self.last_pose = None
        self.last_time = None
        self.last_setpoint = None
        self.last_cmd_vel = None
        self.last_hover = None
        self.arm_requested = False
        self.takeoff_requested = False
        self.takeoff_request_time = None
        self.desired_z = 0.0
        self.pose_count = 0
        self.setpoint_count = 0
        self.cmd_vel_count = 0
        self.hover_count = 0

        self.startup_timer = self.create_timer(0.5, self.startup_sequence)
        self.diagnostics_timer = self.create_timer(2.0, self.log_diagnostics)

        self.get_logger().info(
            f"Hardware bridge: {self.pose_topic} -> {self.odom_topic}"
        )
        self.get_logger().info(
            f"Hardware bridge: {self.cmd_vel_topic} -> {self.cmd_hover_topic}"
        )

    def pose_callback(self, msg):
        self.pose_count += 1
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.robot_name
        odom.pose.pose = msg.pose

        stamp = Time.from_msg(msg.header.stamp)
        if self.last_pose is not None and self.last_time is not None:
            dt = (stamp - self.last_time).nanoseconds * 1e-9
            if dt > 0.0:
                odom.twist.twist.linear.x = (
                    msg.pose.position.x - self.last_pose.position.x
                ) / dt
                odom.twist.twist.linear.y = (
                    msg.pose.position.y - self.last_pose.position.y
                ) / dt
                odom.twist.twist.linear.z = (
                    msg.pose.position.z - self.last_pose.position.z
                ) / dt

        self.last_pose = msg.pose
        self.last_time = stamp
        self.odom_pub.publish(odom)

    def setpoint_callback(self, msg):
        self.setpoint_count += 1
        self.last_setpoint = msg
        self.desired_z = float(msg.pose.position.z)

    def cmd_vel_callback(self, msg):
        self.cmd_vel_count += 1
        self.last_cmd_vel = msg
        if self.desired_z <= 0.05:
            return
        if not self.takeoff_finished():
            return

        hover = Hover()
        hover.header.stamp = self.get_clock().now().to_msg()
        hover.header.frame_id = 'world'
        hover.vx = float(msg.linear.x)
        hover.vy = float(msg.linear.y)
        hover.yaw_rate = float(msg.angular.z)
        hover.z_distance = float(self.desired_z)

        self.last_hover = hover
        self.hover_count += 1
        self.cmd_hover_pub.publish(hover)

    def startup_sequence(self):
        if self.last_setpoint is None or self.desired_z <= 0.05:
            return

        if self.auto_arm and not self.arm_requested:
            if not self.request_arm():
                return

        if self.auto_takeoff and not self.takeoff_requested:
            self.request_takeoff(self.desired_z)

    def request_arm(self):
        if not self.arm_client.service_is_ready():
            self.get_logger().warn(
                f"{self.arm_service_name} is not ready; waiting before arm"
            )
            if not self.arm_client.wait_for_service(timeout_sec=1.0):
                return False

        request = Arm.Request()
        request.arm = True
        self.arm_client.call_async(request)
        self.arm_requested = True
        self.get_logger().info("Requested Crazyflie arm")
        return True

    def request_takeoff(self, height):
        if not self.takeoff_client.service_is_ready():
            self.get_logger().warn(
                f"{self.takeoff_service_name} is not ready; waiting before takeoff"
            )
            if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
                return

        request = Takeoff.Request()
        request.group_mask = 0
        request.height = float(height)
        request.duration = self.seconds_to_duration(self.takeoff_duration)

        self.takeoff_client.call_async(request)
        self.takeoff_requested = True
        self.takeoff_request_time = self.get_clock().now()
        self.get_logger().info(
            f"Requested takeoff to {height:.2f} m over {self.takeoff_duration:.1f} s"
        )

    def takeoff_finished(self):
        if not self.takeoff_requested or self.takeoff_request_time is None:
            return False
        elapsed = (self.get_clock().now() - self.takeoff_request_time).nanoseconds
        return elapsed * 1e-9 >= self.takeoff_duration

    def log_diagnostics(self):
        self.get_logger().info(
            "hardware bridge status | "
            f"pose={self.pose_count} setpoint={self.setpoint_count} "
            f"cmd_vel={self.cmd_vel_count} hover={self.hover_count} "
            f"armed={self.arm_requested} takeoff={self.takeoff_requested} "
            f"z={self.desired_z:.2f}"
        )

    @staticmethod
    def seconds_to_duration(seconds):
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
