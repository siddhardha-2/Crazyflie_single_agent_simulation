#!/usr/bin/env python3

import time

import rclpy
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from rclpy.node import Node


class HardwareHoverTest(Node):
    def __init__(self):
        super().__init__('hardware_hover_test')
        self.declare_parameter('robot_name', 'cf0')
        self.declare_parameter('altitude', 0.5)
        self.declare_parameter('takeoff_duration', 3.0)
        self.declare_parameter('hold_duration', 10.0)
        self.declare_parameter('land_duration', 3.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.altitude = float(self.get_parameter('altitude').value)
        self.takeoff_duration = float(
            self.get_parameter('takeoff_duration').value
        )
        self.hold_duration = float(self.get_parameter('hold_duration').value)
        self.land_duration = float(self.get_parameter('land_duration').value)

        self.takeoff_client = self.create_client(
            Takeoff, f'/{self.robot_name}/takeoff'
        )
        self.goto_client = self.create_client(
            GoTo, f'/{self.robot_name}/go_to'
        )
        self.land_client = self.create_client(
            Land, f'/{self.robot_name}/land'
        )

    def wait_for_services(self):
        self.get_logger().info('Waiting for Crazyflie high-level services...')
        self.takeoff_client.wait_for_service()
        self.goto_client.wait_for_service()
        self.land_client.wait_for_service()

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise future.exception()

    def run(self):
        self.wait_for_services()

        takeoff = Takeoff.Request()
        takeoff.group_mask = 0
        takeoff.height = self.altitude
        takeoff.duration = rclpy.duration.Duration(
            seconds=self.takeoff_duration
        ).to_msg()
        self.get_logger().info(f'Taking off to {self.altitude:.2f} m')
        self.call(self.takeoff_client, takeoff)

        goto = GoTo.Request()
        goto.group_mask = 0
        goto.relative = False
        goto.goal.z = self.altitude
        goto.duration = rclpy.duration.Duration(
            seconds=self.takeoff_duration
        ).to_msg()
        self.call(self.goto_client, goto)

        self.get_logger().info(
            f'Holding at {self.altitude:.2f} m for {self.hold_duration:.1f} s'
        )
        time.sleep(self.hold_duration)

        land = Land.Request()
        land.group_mask = 0
        land.height = 0.0
        land.duration = rclpy.duration.Duration(
            seconds=self.land_duration
        ).to_msg()
        self.get_logger().info('Landing')
        self.call(self.land_client, land)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareHoverTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
