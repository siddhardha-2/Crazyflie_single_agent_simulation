import numpy as np
import rclpy

from Guidance.extremum_seeker import ExtremumSeeker


TARGET_X = 0.5
TARGET_Y = 0.5


class HardwareExtremumSeeker(ExtremumSeeker):
    """ROS hardware profile for the extremum seeker.

    This keeps the algorithm on the same ROS graph as simulation:
    /cf0/state -> /cf0/setpoint. The Crazyflie radio is owned by
    crazyflie_server, so this node must not connect with cflib directly.
    """

    def __init__(self):
        super().__init__()

        self.target = np.array([TARGET_X, TARGET_Y])
        self.w_star = np.array([
            -1.0,
            -1.0,
            2.0 * TARGET_X,
            2.0 * TARGET_Y,
            -(TARGET_X**2 + TARGET_Y**2),
        ])

        self.alg_x = 0.0
        self.alg_y = 0.0
        self.x_min, self.x_max = -1.0, 1.0
        self.y_min, self.y_max = -1.0, 1.0

        self.get_logger().info(
            f'Hardware profile active: target=({TARGET_X:.2f},{TARGET_Y:.2f}), '
            'bounds=[-1,1]m'
        )

    def dynamics_step(self, t, x):
        p = self.phi(x[0], x[1])
        J = self.jac_2(x[0], x[1])
        wp = np.clip(float(self.w_hat @ p), -500, 500)
        fh = 1.0 - np.exp(wp)
        probe = self.probe_amp * np.exp(-self.probe_decay * t) * np.sin(
            self.probe_freq * t
        )
        psi = self.psi_scale * (J.T @ self.w_hat) + probe
        xdot = self.projection_x(x, psi)
        Z1 = float(np.linalg.norm(x - self.target))
        return xdot, fh, Z1


def main(args=None):
    rclpy.init(args=args)
    node = HardwareExtremumSeeker()

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
