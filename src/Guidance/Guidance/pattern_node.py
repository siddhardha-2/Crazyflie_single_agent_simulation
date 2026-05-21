import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from navigation.msg import DroneState
import math


class PatternNode(Node):

    def __init__(self):
        super().__init__('pattern_node')

        # -----------------------------------------------------------------------
        # Load parameters
        # -----------------------------------------------------------------------
        self.declare_parameter('square_size',        1.0)
        self.declare_parameter('altitude',           1.0)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('takeoff_tolerance',  0.1)
        self.declare_parameter('guidance_rate',      20.0)
        self.declare_parameter('loop_pattern',       True)

        self.square_size        = self.get_parameter('square_size').value
        self.altitude           = self.get_parameter('altitude').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.takeoff_tolerance  = self.get_parameter('takeoff_tolerance').value
        self.guidance_rate      = self.get_parameter('guidance_rate').value
        self.loop_pattern       = self.get_parameter('loop_pattern').value

        # -----------------------------------------------------------------------
        # Waypoints — square pattern
        # (0,0) → (1,0) → (1,1) → (0,1) → back to (0,0)
        # -----------------------------------------------------------------------
        s = self.square_size
        z = self.altitude

        self.waypoints = [
            (0.0, 0.0, z),   # takeoff
            (s,   0.0, z),   # corner 1
            (s,   s,   z),   # corner 2
            (0.0, s,   z),   # corner 3
            (0.0, 0.0, z),   # back to start
        ]

        self.current_wp_index = 0
        self.takeoff_done     = False

        # -----------------------------------------------------------------------
        # State
        # -----------------------------------------------------------------------
        self.current_state = None

        # -----------------------------------------------------------------------
        # Subscriber & Publisher
        # -----------------------------------------------------------------------
        self.state_sub = self.create_subscription(
            DroneState,
            '/cf0/state',
            self.state_callback,
            10)

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/cf0/setpoint',
            10)

        # -----------------------------------------------------------------------
        # Timer
        # -----------------------------------------------------------------------
        period = 1.0 / self.guidance_rate
        self.timer = self.create_timer(period, self.guidance_loop)

        self.get_logger().info('PatternNode started — Square pattern')
        self.get_logger().info(
            f'Square size: {self.square_size}m  Altitude: {self.altitude}m')

    # -----------------------------------------------------------------------
    # State Callback
    # -----------------------------------------------------------------------
    def state_callback(self, msg):
        self.current_state = msg

    # -----------------------------------------------------------------------
    # Guidance Loop
    # -----------------------------------------------------------------------
    def guidance_loop(self):
        if self.current_state is None:
            return

        # Current waypoint
        wp = self.waypoints[self.current_wp_index]

        # Publish current waypoint as setpoint
        self.publish_setpoint(wp[0], wp[1], wp[2])

        # Check if drone reached current waypoint
        if self.reached(wp):
            self.get_logger().info(
                f'Reached waypoint {self.current_wp_index}: '
                f'({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})')

            next_index = self.current_wp_index + 1

            # All waypoints done
            if next_index >= len(self.waypoints):
                if self.loop_pattern:
                    # Skip takeoff waypoint on loop (index 1)
                    self.current_wp_index = 1
                    self.get_logger().info('Looping pattern...')
                else:
                    self.get_logger().info('Pattern complete. Holding position.')
            else:
                self.current_wp_index = next_index

    # -----------------------------------------------------------------------
    # Publish Setpoint
    # -----------------------------------------------------------------------
    def publish_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Keep orientation flat (no yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.setpoint_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Check if drone reached waypoint
    # -----------------------------------------------------------------------
    def reached(self, wp):
        dx = self.current_state.x - wp[0]
        dy = self.current_state.y - wp[1]
        dz = self.current_state.z - wp[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < self.waypoint_tolerance


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PatternNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()