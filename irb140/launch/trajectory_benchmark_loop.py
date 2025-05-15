#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import subprocess
import time
import csv
import os


class TrajectoryBenchmarkLoop(Node):
    def __init__(self):
        super().__init__('trajectory_benchmark_loop')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Publish every 5 seconds
        self.timer_ = self.create_timer(5.0, self.publish_trajectory)

        self.start_time = time.time()
        self.max_duration = 60  # Benchmark duration (seconds)
        self.latency_log = []
        self.output_file = f'latency_{os.uname().nodename}.csv'
        self.mbw_log = f'memory_bandwidth_{os.uname().nodename}.txt'

        self.get_logger().info('Benchmark node started...')

    def publish_trajectory(self):
        trajectory_points = [
            [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            [0.3, -0.3, 0.2, 0.1, 0.8, -0.1],
            [0.5,  0.0, -0.5, 0.5, 0.0, -0.5],
            [0.2,  0.2, -0.2, 0.2, -0.2, 0.2],
            [0.0,  0.0,  0.0, 0.0,  0.0, 0.0]
        ]

        traj = JointTrajectory()
        traj.joint_names = [f'joint_{i}' for i in range(1, 7)]

        for i, positions in enumerate(trajectory_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = (i + 1) * 2
            point.time_from_start.nanosec = 0
            traj.points.append(point)

        now = self.get_clock().now()
        traj.header.stamp = now.to_msg()

        # Log publishing timestamp
        self.latency_log.append((now.nanoseconds / 1e6, 'published'))  # ms

        self.publisher_.publish(traj)
        self.get_logger().info('Published trajectory with 5 points.')

        elapsed = time.time() - self.start_time
        if elapsed >= self.max_duration:
            self.get_logger().info('Max duration reached. Saving logs...')
            self.save_latency()
            self.log_memory_bandwidth()
            rclpy.shutdown()

    def save_latency(self):
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time (ms)', 'Event'])
            writer.writerows(self.latency_log)
        self.get_logger().info(f'Latency log saved to {self.output_file}')

    def log_memory_bandwidth(self):
        self.get_logger().info('Running mbw to log memory bandwidth...')
        try:
            result = subprocess.run(
                ['mbw', '-n', '3', '128'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            with open(self.mbw_log, 'w') as f:
                f.write(result.stdout)
            self.get_logger().info(f'Memory bandwidth logged to {self.mbw_log}')
        except Exception as e:
            self.get_logger().error(f'Failed to run mbw: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBenchmarkLoop()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

