import signal
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray
from navigation.msg import DroneState
from crazyflie_interfaces.srv import Land
from builtin_interfaces.msg import Duration


# ---------------------------------------------------------------------------
# Constants — mirror the validated simulation exactly
# ---------------------------------------------------------------------------
SIZE   = 5
Ts     = 0.01          # algorithm / integrator timestep: 100 Hz
ROS_HZ = 50.0          # ROS timer rate: 50 Hz  →  2 sub-steps per tick
N_STEPS = int((1.0 / ROS_HZ) / Ts)   # = 2

W_STAR  = np.array([1.0, 1.5, 0.0, -4.5, 3.375])
W_HAT0  = np.array([0.1, 0.1, 0.1, 0.1, 0.1])

GAMMA   = 0.5
K_CTRL  = 0.52

PROBE_A = 0.35
PROBE_B = -1.35
PROBE_C = 2.4

DECAY   = 0.2          # exponential decay in matrix1
CL_C1   = 1.115        # concurrent-learning coefficient
CL_C2   = 4.45


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class HardwareExtremumSeekerTV(Node):
    """
    ROS2 hardware node for the time-varying extremum seeker.

    Algorithm is a direct port of Time_varying_extremum_CF.py.
    Hardware-specific additions:
      - Lighthouse auto-calibration (finds physical center before flying)
      - Takeoff phase (holds starting position for 3 s)
      - Safety-bounds clipping in absolute frame
      - Safety land on Ctrl-C
    """

    def __init__(self):
        super().__init__('extremum_seeker_tv_hardware')

        # ------------------------------------------------------------------ #
        # Algorithm state — identical to the simulation script               #
        # ------------------------------------------------------------------ #
        self.w_hatd = W_HAT0.copy()
        self.Yd     = np.zeros((SIZE, SIZE))
        self.df     = np.zeros(SIZE)
        self.p      = np.array([0.5, 0.5])   # relative position (x, y)
        self.t      = 0.0

        # ------------------------------------------------------------------ #
        # Hardware / flight state                                             #
        # ------------------------------------------------------------------ #
        self.drone_x   = 0.0
        self.drone_y   = 0.0
        self.has_state = False

        # Center detected by Lighthouse calibration
        self.center_x = 0.0
        self.center_y = 0.0

        # Safety bounds (absolute frame, set after calibration)
        self.x_min = -999.0
        self.x_max =  999.0
        self.y_min = -999.0
        self.y_max =  999.0

        # Physical safety margins around the detected center
        self.bound_radius_x     = 1.0
        self.bound_radius_y_neg = 0.5
        self.bound_radius_y_pos = 1.6

        # Flight altitude
        self.altitude = 0.5

        # ------------------------------------------------------------------ #
        # Phase flags                                                         #
        # ------------------------------------------------------------------ #
        self.calibrated   = False
        self.takeoff_done = False
        self.takeoff_t    = 0.0          # separate counter for takeoff phase

        # Calibration settings
        self.calib_readings   = []
        self.calib_count      = 10
        self.calib_tolerance  = 0.3

        # ------------------------------------------------------------------ #
        # Logging state (for debug publishers)                                #
        # ------------------------------------------------------------------ #
        self.last_u   = np.zeros(2)
        self.last_Z1  = 0.0
        self.last_Z11 = 0.0

        # ------------------------------------------------------------------ #
        # ROS subscribers, publishers, services                               #
        # ------------------------------------------------------------------ #
        self.state_sub = self.create_subscription(
            DroneState, '/cf0/state', self._state_cb, 10)

        self.sp_pub    = self.create_publisher(PoseStamped,       '/cf0/setpoint',    10)
        self.fhat_pub  = self.create_publisher(Float64,           '/cf0/f_hat',       10)
        self.z1_pub    = self.create_publisher(Float64,           '/cf0/Z1',          10)
        self.z11_pub   = self.create_publisher(Float64,           '/cf0/Z11',         10)
        self.xdot_pub  = self.create_publisher(Float64MultiArray, '/cf0/x_dot',       10)
        self.what_pub  = self.create_publisher(Float64MultiArray, '/cf0/w_hat_debug', 10)

        self.land_client = self.create_client(Land, '/cf0/land')

        self.timer = self.create_timer(1.0 / ROS_HZ, self._loop)

        self.get_logger().info(
            'HardwareExtremumSeekerTV started — waiting for Lighthouse position...')

    # ====================================================================== #
    # Subscriber callbacks                                                    #
    # ====================================================================== #

    def _state_cb(self, msg):
        self.drone_x   = msg.x
        self.drone_y   = msg.y
        self.has_state = True

        # Collect raw readings for calibration
        if not self.calibrated:
            self.calib_readings.append((msg.x, msg.y))

    # ====================================================================== #
    # Main timer loop                                                         #
    # ====================================================================== #

    def _loop(self):
        # --- Phase 0: wait for first pose data ---
        if not self.has_state:
            return

        # --- Phase 1: Lighthouse calibration ---
        if not self.calibrated:
            self._calibration_phase()
            # Hold current position during calibration
            self._publish(self.drone_x, self.drone_y)
            return

        # --- Phase 2: Takeoff / fly to starting point ---
        if not self.takeoff_done:
            self._takeoff_phase()
            return

        # --- Phase 3: Algorithm (direct port of simulation loop) ---
        self._algorithm_loop()

    # ====================================================================== #
    # Phase helpers                                                           #
    # ====================================================================== #

    def _calibration_phase(self):
        if len(self.calib_readings) < self.calib_count:
            return

        recent = self.calib_readings[-self.calib_count:]
        xs = [r[0] for r in recent]
        ys = [r[1] for r in recent]
        spread_x = max(xs) - min(xs)
        spread_y = max(ys) - min(ys)

        if spread_x < self.calib_tolerance and spread_y < self.calib_tolerance:
            self.center_x = float(np.median(xs))
            self.center_y = float(np.median(ys))

            self.x_min = self.center_x - self.bound_radius_x
            self.x_max = self.center_x + self.bound_radius_x
            self.y_min = self.center_y - self.bound_radius_y_neg
            self.y_max = self.center_y + self.bound_radius_y_pos

            self.calibrated = True
            self.get_logger().info(
                f'Calibrated! center=({self.center_x:.3f}, {self.center_y:.3f}) | '
                f'X[{self.x_min:.2f},{self.x_max:.2f}] '
                f'Y[{self.y_min:.2f},{self.y_max:.2f}]')
        else:
            self.get_logger().info(
                f'Waiting for stable Lighthouse... spread=({spread_x:.3f},{spread_y:.3f})',
                throttle_duration_sec=2.0)

    def _takeoff_phase(self):
        """Hold the algorithm start position (relative 0.5,0.5) for 3 seconds."""
        start_x = 0.5 + self.center_x
        start_y = 0.5 + self.center_y
        self._publish(start_x, start_y)

        self.takeoff_t += 1.0 / ROS_HZ
        if self.takeoff_t >= 3.0:
            self.takeoff_done = True
            # Sync algorithm position to the starting point
            self.p = np.array([0.5, 0.5])
            self.get_logger().info(
                f'Takeoff done — algorithm starting at ({start_x:.3f},{start_y:.3f})')

    def _algorithm_loop(self):
        """
        Direct port of Time_varying_extremum_CF.py loop body.
        Two sub-steps per ROS tick (100 Hz algorithm inside 50 Hz timer).
        """
        for _ in range(N_STEPS):
            t = self.t
            x, y = self.p[0], self.p[1]

            a1, a2, deltaTheta = self._get_tv_params(t)

            # ---- Estimator (Concurrent Learning) ---- #
            phiVec  = np.array([x**2, y**2, x, y, 1.0])
            decay   = np.exp(-DECAY * t)
            matrix1 = np.outer(phiVec, phiVec) * decay

            wtilda        = self.w_hatd - W_STAR
            phiPhi_wtilda = matrix1 @ wtilda
            phiPhi_delta  = matrix1 @ deltaTheta
            Yd_wtilda     = self.Yd @ wtilda

            yVec = (-CL_C1 * phiPhi_wtilda
                    - CL_C2 * Yd_wtilda
                    + CL_C2 * self.df
                    + CL_C1 * phiPhi_delta)

            self.w_hatd += (GAMMA * yVec) * Ts
            self.Yd     += matrix1 * Ts
            self.df     += (matrix1 @ deltaTheta) * Ts

            # ---- Control (Dynamics) ---- #
            jac2 = np.array([
                [2.0 * x, 0.0],
                [0.0,     2.0 * y],
                [1.0,     0.0],
                [0.0,     1.0],
                [0.0,     0.0]
            ])

            probe = np.array([
                PROBE_A * np.exp(PROBE_B * t),
                PROBE_A * np.exp(PROBE_B * t) * np.sin(PROBE_C * t)
            ])

            u = -K_CTRL * (self.w_hatd @ jac2) + probe

            # ---- Euler integration ---- #
            self.p += u * Ts

            # ---- Safety bounds (absolute frame) ---- #
            abs_x = np.clip(self.p[0] + self.center_x, self.x_min, self.x_max)
            abs_y = np.clip(self.p[1] + self.center_y, self.y_min, self.y_max)
            self.p[0] = abs_x - self.center_x
            self.p[1] = abs_y - self.center_y

            # ---- Advance time ---- #
            self.t += Ts

            # Keep last step values for logging
            self.last_u   = u.copy()
            self.last_Z1  = float(np.linalg.norm(self.p - np.array([a1, a2])))
            self.last_Z11 = float(np.linalg.norm(wtilda))

        # ---- Publish setpoint (once per ROS tick, after sub-steps) ---- #
        abs_x_pub = self.p[0] + self.center_x
        abs_y_pub = self.p[1] + self.center_y
        self._publish(abs_x_pub, abs_y_pub)

        # ---- Debug publishers ---- #
        x_now  = self.p.copy()
        phi_now = np.array([x_now[0]**2, x_now[1]**2, x_now[0], x_now[1], 1.0])
        fhat_val = float(self.w_hatd @ phi_now)

        m = Float64(); m.data = fhat_val;       self.fhat_pub.publish(m)
        m = Float64(); m.data = self.last_Z1;   self.z1_pub.publish(m)
        m = Float64(); m.data = self.last_Z11;  self.z11_pub.publish(m)

        xd = Float64MultiArray()
        xd.data = [float(self.last_u[0]), float(self.last_u[1])]
        self.xdot_pub.publish(xd)

        wm = Float64MultiArray()
        wm.data = [float(v) for v in self.w_hatd]
        self.what_pub.publish(wm)

        self.get_logger().info(
            f't={self.t:.2f}s | '
            f'pos=({abs_x_pub:.3f},{abs_y_pub:.3f}) | '
            f'drone=({self.drone_x:.3f},{self.drone_y:.3f}) | '
            f'u=({self.last_u[0]:.4f},{self.last_u[1]:.4f}) | '
            f'Z1={self.last_Z1:.4f} Z11={self.last_Z11:.4f}',
            throttle_duration_sec=1.0)

    # ====================================================================== #
    # Algorithm helpers                                                       #
    # ====================================================================== #

    @staticmethod
    def _get_tv_params(t):
        """Time-varying target and deltaTheta — identical to simulation."""
        a1      = 0.5 * np.exp(-0.1 * t) * np.sin(0.5 * t)
        term_a2 = 0.1 * np.exp(-0.05 * t) * np.cos(0.3 * t)
        a2      = 1.5 - term_a2

        deltaTheta    = np.zeros(SIZE)
        deltaTheta[2] = -2.0 * a1
        deltaTheta[3] = 0.3 * np.exp(-0.05 * t) * np.cos(0.3 * t)
        deltaTheta[4] = a1**2 + 1.5 * (term_a2**2) - deltaTheta[3]
        return a1, a2, deltaTheta

    # ====================================================================== #
    # ROS helpers                                                             #
    # ====================================================================== #

    def _publish(self, x, y):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(self.altitude)
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)

    def safety_land(self):
        """Send a Land service call — called on Ctrl-C."""
        self.get_logger().warn('Safety landing initiated...')
        try:
            if not rclpy.ok():
                print('[WARN] ROS context already shut down — cannot send land.')
                return
            if not self.land_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Land service unavailable — drone may still be flying!')
                return
            req          = Land.Request()
            req.group_mask = 0
            req.height   = 0.0
            req.duration = Duration(sec=3, nanosec=0)
            future = self.land_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info('Land command sent.')
        except Exception as e:
            print(f'[ERROR] safety_land: {e}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HardwareExtremumSeekerTV()

    shutdown_requested = [False]

    def _sigint(sig, frame):
        if not shutdown_requested[0]:
            shutdown_requested[0] = True
            node.safety_land()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
