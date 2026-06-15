"""
extremum_seeker_tv_proposed.py
================================
ROS 2 Guidance node implementing the PROPOSED Time-Varying Extremum Seeking
algorithm (Dynamic Projection + Parameter Estimation + CONCURRENT LEARNING).
"""

import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

try:
    from navigation.msg import DroneState
except ImportError:
    DroneState = None

SIZE = 5  

class ExtremumSeekerTVProposed(Node):

    def __init__(self):
        super().__init__('extremum_seeker_tv_proposed')

        self.declare_parameter('timer_period',  0.02)
        self.declare_parameter('hover_z',       0.5)

        self._Ts      = self.get_parameter('timer_period').value
        self._hover_z = self.get_parameter('hover_z').value

        self._w_star = np.array([-1.0, -0.5, 4.0, 2.0, -6.0], dtype=float)
        self._w_hat  = np.array([0.1, 0.1, 0.1, 0.1, 0.1], dtype=float)
        self._p      = np.array([0.5, 0.5], dtype=float) 
        
        self._Yd = np.zeros((SIZE, SIZE))
        self._yd = np.zeros(SIZE)
        
        self._t = 0.0
        self._state_received = False 

        # ---> UNIQUE CSV FOR PROPOSED ALGORITHM <---
        csv_path = os.path.expanduser('~/internship_ws/flight_data_tv_proposed.csv')
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['t', 'x', 'y', 'ref_x', 'ref_y', 'Z1', 'Z11'])

        if DroneState is not None:
            self._state_sub = self.create_subscription(
                DroneState, '/cf0/state', self._state_callback, 10)
        else:
            self.get_logger().warn('navigation.msg.DroneState not found.')

        self._odom_sub = self.create_subscription(
            Odometry, '/cf0/odom', self._odom_callback, 10)

        self._setpoint_pub = self.create_publisher(PoseStamped, '/cf0/setpoint', 10)
        self._debug_pub    = self.create_publisher(Float64MultiArray, '/cf0/es_debug', 10)

        self._timer = self.create_timer(self._Ts, self._timer_callback)

        self.get_logger().info(f'ExtremumSeekerTVProposed started. Logging to {csv_path}')

    def destroy_node(self):
        if hasattr(self, '_csv_file'):
            self._csv_file.close()
        super().destroy_node()

    def _state_callback(self, msg):
        self._state_received = True

    def _odom_callback(self, msg: Odometry):
        if not self._state_received:
            self._state_received = True

    def _timer_callback(self):
        if not self._state_received:
            self.get_logger().warn('Waiting for hardware connection...', throttle_duration_sec=2.0)
            return

        t  = self._t
        Ts = self._Ts
        x, y = self._p[0], self._p[1]

        phiVec = np.array([x**2, y**2, x, y, 1.0])

        delta_theta = self._delta_theta(t)
        wtilda = self._w_hat - (self._w_star + delta_theta)

        f_true = 1.0 - np.exp((self._w_star + delta_theta) @ phiVec)
        y_meas = np.log(max(1.0 - f_true, 1e-12))

        f_hat = 1.0 - np.exp(self._w_hat @ phiVec)
        y_hat = np.log(max(1.0 - f_hat, 1e-12))

        decay = np.exp(-0.03 * t)
        Yd_dot = decay * np.outer(phiVec, phiVec)
        yd_dot = decay * (phiVec * y_meas)

        self._Yd += Yd_dot * Ts
        self._yd += yd_dot * Ts

        Gamma = 0.08 * np.eye(SIZE)
        
        # ---> CONCURRENT LEARNING IS ACTIVE HERE <---
        y_law = -0.25 * phiVec * (y_hat - y_meas) - 0.75 * (self._Yd @ self._w_hat - self._yd)

        alpha = 18.5
        epsilon = 0.5
        den_proj_math = 2.0 * alpha * epsilon + epsilon**2
        f_proj = (np.linalg.norm(self._w_hat)**2 - alpha**2) / den_proj_math
        gradf_proj = (2.0 * self._w_hat) / den_proj_math

        num_proj = gradf_proj @ Gamma @ y_law
        den_proj = gradf_proj @ Gamma @ gradf_proj

        if (f_proj > 0) and (num_proj > 0) and (den_proj > 1e-11):
            corr = (Gamma @ gradf_proj) * (num_proj / den_proj) * f_proj
            w_hat_dot = Gamma @ y_law - corr
        else:
            w_hat_dot = Gamma @ y_law

        self._w_hat += w_hat_dot * Ts

        jac2 = np.array([
            [2.0 * x, 0.0],
            [0.0,     2.0 * y],
            [1.0,     0.0],
            [0.0,     1.0],
            [0.0,     0.0]
        ])

        probe = np.array([
            0.35 * np.exp(-0.35 * t), 
            0.35 * np.exp(-0.35 * t) * np.sin(0.4 * t)
        ])
        
        psi_x = 0.82 * (self._w_hat @ jac2) + probe

        epsilon1 = 0.3
        alpha1 = 5.5
        den1 = 2.0 * alpha1 * epsilon1 + epsilon1**2
        f1 = (np.linalg.norm(self._p)**2 - alpha1**2) / den1
        gradf1 = (2.0 / den1) * self._p

        Gamma_x = 1.0
        ytGgrad = psi_x @ (Gamma_x * gradf1)
        denom = (gradf1 @ gradf1) * Gamma_x

        if (f1 > 0) and (ytGgrad > 0) and (denom > 1e-12):
            corr1 = gradf1 * ((gradf1 @ psi_x) / denom) * f1 * Gamma_x
            p_dot = Gamma_x * psi_x - corr1
        else:
            p_dot = Gamma_x * psi_x

        self._p += p_dot * Ts

        ref_x = 2.0 + 0.1 * np.sin(0.2 * t)
        ref_y = 2.0 - 0.1 * np.sin(t)
        lo = self._p - np.array([ref_x, ref_y])
        
        Z1 = np.linalg.norm(lo, 2)
        Z11 = np.linalg.norm(wtilda)

        self._csv_writer.writerow([
            t, self._p[0], self._p[1], ref_x, ref_y, float(Z1), float(Z11)
        ])

        setpoint_msg = PoseStamped()
        setpoint_msg.header.stamp    = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = 'world'
        setpoint_msg.pose.position.x = float(self._p[0])
        setpoint_msg.pose.position.y = float(self._p[1])
        setpoint_msg.pose.position.z = float(self._hover_z)
        setpoint_msg.pose.orientation.w = 1.0
        self._setpoint_pub.publish(setpoint_msg)

        dbg      = Float64MultiArray()
        dbg.data = list(self._w_hat) + [float(Z1), float(Z11), float(f_proj), float(f1)]
        self._debug_pub.publish(dbg)

        self._t += Ts

    def _delta_theta(self, t: float) -> np.ndarray:
        d = np.zeros(SIZE)
        d[2] = 0.2  * np.sin(0.2 * t)
        d[3] = -0.1 * np.sin(t)
        d[4] = (
            -0.4   * np.sin(0.2 * t)
            + 0.2  * np.sin(t)
            - 0.01 * np.sin(0.2 * t)**2
            - 0.005* np.sin(t)**2
        )
        return d

def main(args=None):
    rclpy.init(args=args)
    node = ExtremumSeekerTVProposed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
