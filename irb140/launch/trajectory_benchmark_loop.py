#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import csv
import os
import time

class JitterBenchmarkNode(Node):
    def __init__(self):
        super().__init__('trajectory_jitter_node')

        self.timer_period = 0.01  # 10ms = 100Hz
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

        self.expected_time = self.get_clock().now().nanoseconds / 1e9
        self.jitter_log = []

        self.max_duration = 10.0  # seconds
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.output_file = f'jitter_log_{os.uname().nodename}.csv'
        self.logger = self.get_logger()
        self.logger.info('Started jitter logging benchmark.')

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        # Compute jitter = actual_time - expected_time
        jitter = (now - self.expected_time) * 1e6  # Convert to microseconds
        self.jitter_log.append(jitter)
        self.expected_time += self.timer_period  # Update expected time for next cycle

        # Publish simple dummy trajectory
        traj = JointTrajectory()
        traj.joint_names = [f'joint_{i+1}' for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5, 0.5, 0.0, 1.0, 0.0]
        point.time_from_start.sec = 1
        traj.points.append(point)
        traj.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(traj)

        self.logger.info(f'Published trajectory. Jitter: {jitter:.2f} µs')

        # Stop after max_duration
        if elapsed >= self.max_duration:
            self.save_results()
            rclpy.shutdown()

    def save_results(self):
        with open(self.output_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Jitter (microseconds)'])
            for value in self.jitter_log:
                writer.writerow([value])
        self.logger.info(f'Jitter data saved to {self.output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = JitterBenchmarkNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

