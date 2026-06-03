"""
extremum_seeker.py — faithful port of PL_EXTREMUM.slx

Sub-stepping: algorithm runs at dt=0.001s (Simulink rate) internally,
publishes setpoint to ROS2 every 0.02s (50Hz).
This matches Simulink's numerical behavior exactly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Float64
from navigation.msg import DroneState
import numpy as np


class ExtremumSeeker(Node):

    def __init__(self):
        super().__init__('extremum_seeker')

        # -----------------------------------------------------------------------
        # Exact Simulink parameters — unchanged
        # -----------------------------------------------------------------------
        self.w_star    = np.array([-1.0, -0.5, 4.0, 2.0, -6.0])
        self.Gamma_w   = 1.5 * np.eye(5)
        self.alpha_w   = 18.5
        self.epsilon_w = 0.5
        self.k1        = 3.5
        self.k2        = 15.5       # original Simulink value — no scaling needed
        self.lambda_f  = 0.03
        self.Gamma_x   = 1.0
        self.alpha_x   = 5.5
        self.epsilon_x = 0.3
        self.probe_amp   = 0.35
        self.probe_decay = 0.35
        self.probe_freq  = 0.4
        self.psi_scale   = 0.52

        # -----------------------------------------------------------------------
        # Sub-stepping to match Simulink dt=0.001s
        # ROS2 runs at 50Hz (dt=0.02s)
        # Internally we sub-step at dt_sim=0.001s (20 sub-steps per ROS2 tick)
        # -----------------------------------------------------------------------
        self.ros_rate  = 50.0
        self.dt_ros    = 1.0 / self.ros_rate   # 0.02s — ROS2 publish rate
        self.dt_sim    = 0.001                  # Simulink timestep
        self.n_steps   = int(self.dt_ros / self.dt_sim)  # 20 sub-steps

        # -----------------------------------------------------------------------
        # Integrator states — exact Simulink initial conditions
        # -----------------------------------------------------------------------
        self.alg_x = 0.2     # Integrator1 x0[0]
        self.alg_y = -0.1    # Integrator1 x0[1]
        self.w_hat = np.zeros(5)           # Integrator4
        self.Yd    = 1e-4 * np.eye(5)     # Integrator5
        self.yd    = np.zeros(5)           # Integrator2

        # -----------------------------------------------------------------------
        # State
        # -----------------------------------------------------------------------
        self.drone_x      = 0.0
        self.drone_y      = 0.0
        self.has_state    = False
        self.t            = 0.0
        self.takeoff_done = False

        self.declare_parameter('altitude', 1.0)
        self.altitude = self.get_parameter('altitude').value

        self.x_min, self.x_max = -2.0, 4.0
        self.y_min, self.y_max = -4.0, 4.0

        # For logging — store latest values
        self.last_fhat = 0.0
        self.last_Z1   = 0.0
        self.last_Z11  = 0.0
        self.last_xdot = np.zeros(2)

        # -----------------------------------------------------------------------
        # Pub / Sub
        # -----------------------------------------------------------------------
        self.state_sub = self.create_subscription(
            DroneState, '/cf0/state', self.state_cb, 10)
        self.sp_pub   = self.create_publisher(PoseStamped,       '/cf0/setpoint',    10)
        self.fhat_pub = self.create_publisher(Float64,           '/cf0/f_hat',       10)
        self.z1_pub   = self.create_publisher(Float64,           '/cf0/Z1',          10)
        self.z11_pub  = self.create_publisher(Float64,           '/cf0/Z11',         10)
        self.xdot_pub = self.create_publisher(Float64MultiArray, '/cf0/x_dot',       10)
        self.what_pub = self.create_publisher(Float64MultiArray, '/cf0/w_hat_debug', 10)

        self.timer = self.create_timer(self.dt_ros, self.loop)

        self.get_logger().info('ExtremumSeeker — PL_EXTREMUM.slx port')
        self.get_logger().info(f'w_star={self.w_star}')
        self.get_logger().info(
            f'k1={self.k1} k2={self.k2} Gamma_w=1.5 '
            f'alpha_w={self.alpha_w} lambda={self.lambda_f}')
        self.get_logger().info(
            f'Sub-stepping: {self.n_steps} steps per tick '
            f'(dt_sim={self.dt_sim}s = Simulink rate)')

    def state_cb(self, msg):
        self.drone_x   = msg.x
        self.drone_y   = msg.y
        self.has_state = True

    # -----------------------------------------------------------------------
    # Simulink functions — exact
    # -----------------------------------------------------------------------
    def phi(self, x1, x2):
        return np.array([x1**2, x2**2, x1, x2, 1.0])

    def jac_2(self, x1, x2):
        return np.array([
            [2*x1, 0.0],
            [0.0, 2*x2],
            [1.0,  0.0],
            [0.0,  1.0],
            [0.0,  0.0]
        ])

    def projection_w(self, w, g):
        d  = 2*self.alpha_w*self.epsilon_w + self.epsilon_w**2
        f  = (float(w@w) - self.alpha_w**2) / d
        gf = 2*w / d
        n  = float(gf @ (self.Gamma_w @ g))
        dn = float(gf @ (self.Gamma_w @ gf))
        if f > 0 and n > 0 and dn > 1e-11:
            return self.Gamma_w@g - (self.Gamma_w@gf)*(n/dn)*f
        return self.Gamma_w @ g

    def projection_x(self, x, psi):
        d  = 2*self.alpha_x*self.epsilon_x + self.epsilon_x**2
        f  = (float(x@x) - self.alpha_x**2) / d
        gf = 2*x / d
        n  = float(psi @ (self.Gamma_x * gf))
        dn = float(gf @ (self.Gamma_x * gf))
        if f > 0 and n > 0 and dn > 1e-12:
            return self.Gamma_x*psi - gf*(float(gf@psi)/dn)*f*self.Gamma_x
        return self.Gamma_x * psi

    def estimator_step(self, t, x, dt):
        p  = self.phi(x[0], x[1])
        d  = 0.000003 * np.sin(1.1 * t)
        y  = float(self.w_star @ p) + d

        decay  = np.exp(-self.lambda_f * t)
        self.Yd += decay * np.outer(p, p) * dt
        self.Yd += 1e-8 * np.eye(5)
        self.yd += decay * p * y * dt

        wp = np.clip(float(self.w_hat @ p), -500, 500)
        fh = np.clip(1.0 - np.exp(wp), -1e6, 1.0 - 1e-9)
        yh = np.log(1.0 - fh)

        grad = (-self.k1 * p * (yh - y)
                - self.k2 * (self.Yd @ self.w_hat - self.yd))

        w_dot = self.projection_w(self.w_hat, grad)
        w_dot = np.clip(w_dot, -100.0, 100.0)
        self.w_hat += w_dot * dt
        self.w_hat  = np.clip(self.w_hat, -self.alpha_w, self.alpha_w)

        Z11 = float(np.linalg.norm(self.w_hat - self.w_star))
        return fh, Z11

    def dynamics_step(self, t, x):
        p     = self.phi(x[0], x[1])
        J     = self.jac_2(x[0], x[1])
        wp    = np.clip(float(self.w_hat @ p), -500, 500)
        fh    = 1.0 - np.exp(wp)
        probe = self.probe_amp * np.exp(-self.probe_decay*t) * np.sin(self.probe_freq*t)
        psi   = self.psi_scale * (J.T @ self.w_hat) + probe
        xdot  = self.projection_x(x, psi)
        Z1    = float(np.linalg.norm(x - np.array([2.0, 2.0])))
        return xdot, fh, Z1

    # -----------------------------------------------------------------------
    # Main loop — runs at 50Hz
    # -----------------------------------------------------------------------
    def loop(self):
        if not self.has_state:
            self.publish(0.0, 0.0)
            return

        if not self.takeoff_done:
            self.publish(0.0, 0.0)
            if self.t > 3.0:
                self.takeoff_done = True
                self.alg_x = self.drone_x
                self.alg_y = self.drone_y
                self.get_logger().info(
                    f'Takeoff done — alg=({self.alg_x:.3f},{self.alg_y:.3f})')
            self.t += self.dt_ros
            return

        # -----------------------------------------------------------------------
        # Sub-stepping loop — runs n_steps=20 iterations at dt_sim=0.001s
        # This matches Simulink's solver timestep exactly
        # -----------------------------------------------------------------------
        fhat_last = self.last_fhat
        Z11_last  = self.last_Z11
        Z1_last   = self.last_Z1
        xdot_last = self.last_xdot.copy()

        for _ in range(self.n_steps):
            x = np.array([self.alg_x, self.alg_y])

            # Estimator (Integrators 2, 4, 5)
            fhat, Z11 = self.estimator_step(self.t, x, self.dt_sim)

            # Dynamics (Integrator 1)
            xdot, fhat, Z1 = self.dynamics_step(self.t, x)

            # Integrate Integrator1 at Simulink timestep
            self.alg_x = float(np.clip(
                self.alg_x + xdot[0] * self.dt_sim, self.x_min, self.x_max))
            self.alg_y = float(np.clip(
                self.alg_y + xdot[1] * self.dt_sim, self.y_min, self.y_max))

            self.t += self.dt_sim
            fhat_last = fhat
            Z11_last  = Z11
            Z1_last   = Z1
            xdot_last = xdot.copy()

        # Store for logging
        self.last_fhat = fhat_last
        self.last_Z1   = Z1_last
        self.last_Z11  = Z11_last
        self.last_xdot = xdot_last

        # Publish setpoint to drone (50Hz)
        self.publish(self.alg_x, self.alg_y)

        # Publish debug topics
        m = Float64(); m.data = float(fhat_last);  self.fhat_pub.publish(m)
        m = Float64(); m.data = float(Z1_last);    self.z1_pub.publish(m)
        m = Float64(); m.data = float(Z11_last);   self.z11_pub.publish(m)

        xd = Float64MultiArray()
        xd.data = [float(xdot_last[0]), float(xdot_last[1])]
        self.xdot_pub.publish(xd)

        wm = Float64MultiArray()
        wm.data = [float(v) for v in self.w_hat]
        self.what_pub.publish(wm)

        self.get_logger().info(
            f't={self.t:.1f}s | '
            f'alg=({self.alg_x:.3f},{self.alg_y:.3f}) | '
            f'drone=({self.drone_x:.3f},{self.drone_y:.3f}) | '
            f'xdot=({xdot_last[0]:.4f},{xdot_last[1]:.4f}) | '
            f'f_hat={fhat_last:.4f} | '
            f'Z1={Z1_last:.4f} | Z11={Z11_last:.4f}',
            throttle_duration_sec=1.0)

    def publish(self, x, y):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(self.altitude)
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExtremumSeeker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
