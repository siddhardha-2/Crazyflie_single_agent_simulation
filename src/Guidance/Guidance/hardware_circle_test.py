#!/usr/bin/env python3

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class HardwareCircleTest(Node):
    def __init__(self):
        super().__init__('hardware_circle_test')
        self.declare_parameter('robot_name', 'cf0')
        self.declare_parameter('altitude', 0.8)
        self.declare_parameter('radius', 0.8)
        self.declare_parameter('waypoints', 36)
        self.declare_parameter('time_per_point', 0.5)
        self.declare_parameter('takeoff_duration', 3.0)
        self.declare_parameter('land_duration', 3.0)
        self.declare_parameter('pose_timeout', 10.0)
        self.declare_parameter('output_directory', 'experiment_logs')

        self.robot_name = self.get_parameter('robot_name').value
        self.altitude = float(self.get_parameter('altitude').value)
        self.radius = float(self.get_parameter('radius').value)
        self.waypoints = int(self.get_parameter('waypoints').value)
        self.time_per_point = float(
            self.get_parameter('time_per_point').value
        )
        self.takeoff_duration = float(
            self.get_parameter('takeoff_duration').value
        )
        self.land_duration = float(self.get_parameter('land_duration').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.output_directory = Path(
            self.get_parameter('output_directory').value
        ).expanduser()

        self.takeoff_client = self.create_client(
            Takeoff, f'/{self.robot_name}/takeoff'
        )
        self.goto_client = self.create_client(
            GoTo, f'/{self.robot_name}/go_to'
        )
        self.land_client = self.create_client(
            Land, f'/{self.robot_name}/land'
        )
        self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/pose',
            self.pose_callback,
            10,
        )

        self.target = [0.0, 0.0, 0.0]
        self.has_pose = False
        self.airborne = False
        self.log_data = []
        self.start_time = time.monotonic()

    def pose_callback(self, msg):
        self.has_pose = True
        self.log_data.append([
            time.monotonic() - self.start_time,
            *self.target,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def wait_for_ready(self):
        self.takeoff_client.wait_for_service()
        self.goto_client.wait_for_service()
        self.land_client.wait_for_service()

        deadline = time.monotonic() + self.pose_timeout
        while not self.has_pose and time.monotonic() < deadline:
            time.sleep(0.1)
        if not self.has_pose:
            raise RuntimeError(
                f'No Lighthouse pose received on /{self.robot_name}/pose'
            )

    def call(self, client, request):
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.05)
        if future.exception() is not None:
            raise future.exception()

    def takeoff(self):
        self.target = [0.0, 0.0, self.altitude]
        request = Takeoff.Request()
        request.group_mask = 0
        request.height = self.altitude
        request.duration = rclpy.duration.Duration(
            seconds=self.takeoff_duration
        ).to_msg()
        self.call(self.takeoff_client, request)
        self.airborne = True
        time.sleep(self.takeoff_duration + 1.0)

    def go_to(self, x, y, duration):
        self.target = [x, y, self.altitude]
        request = GoTo.Request()
        request.group_mask = 0
        request.relative = False
        request.goal.x = float(x)
        request.goal.y = float(y)
        request.goal.z = self.altitude
        request.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.call(self.goto_client, request)

    def land(self):
        self.target[2] = 0.0
        request = Land.Request()
        request.group_mask = 0
        request.height = 0.0
        request.duration = rclpy.duration.Duration(
            seconds=self.land_duration
        ).to_msg()
        self.call(self.land_client, request)
        self.airborne = False
        time.sleep(self.land_duration + 1.0)

    def run(self):
        self.wait_for_ready()
        try:
            self.takeoff()
            self.go_to(self.radius, 0.0, self.takeoff_duration)
            time.sleep(self.takeoff_duration + 0.5)

            for index in range(1, self.waypoints + 1):
                angle = index * 2.0 * math.pi / self.waypoints
                self.go_to(
                    self.radius * math.cos(angle),
                    self.radius * math.sin(angle),
                    self.time_per_point,
                )
                time.sleep(self.time_per_point)

            self.go_to(0.0, 0.0, self.takeoff_duration)
            time.sleep(self.takeoff_duration + 0.5)
            self.land()
            self.export_results()
        finally:
            if self.airborne:
                self.get_logger().warning('Experiment interrupted; landing')
                self.land()

    def export_results(self):
        self.output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = self.output_directory / f'circle_log_{timestamp}.csv'
        plot_path = self.output_directory / f'circle_plot_{timestamp}.png'

        with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'time_s', 'target_x', 'target_y', 'target_z',
                'actual_x', 'actual_y', 'actual_z',
            ])
            writer.writerows(self.log_data)

        plt.figure(figsize=(8, 8))
        plt.plot(
            [row[1] for row in self.log_data],
            [row[2] for row in self.log_data],
            'r--',
            label='Target',
        )
        plt.plot(
            [row[4] for row in self.log_data],
            [row[5] for row in self.log_data],
            'b-',
            label='Lighthouse pose',
        )
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.savefig(plot_path)
        plt.close()
        self.get_logger().info(f'Wrote {csv_path} and {plot_path}')


def main(args=None):
    rclpy.init(args=args)
    node = HardwareCircleTest()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()
    try:
        node.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()
        node.destroy_node()


if __name__ == '__main__':
    main()
