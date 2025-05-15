#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import csv
import os

class RTLoopMonitorNode(Node):
    def __init__(self):
        super().__init__('rt_loop_monitor_benchmark')
        
        # ROS 2 publisher
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Expected loop frequency
        self.loop_period_expected = 5.0  # seconds
        self.last_loop_time = self.get_clock().now().nanoseconds / 1e9
        self.jitter_log = []

        # Start periodic timer
        self.timer = self.create_timer(self.loop_period_expected, self.publish_trajectory)

        self.start_time = time.time()
        self.max_duration = 60  # seconds
        self.output_file = f'rtloop_jitter_{os.uname().nodename}.csv'

        self.get_logger().info('RTLoopMonitor Benchmark started...')

    def publish_trajectory(self):
        now = self.get_clock().now().nanoseconds / 1e9
        jitter = now - self.last_loop_time - self.loop_period_expected
        self.jitter_log.append((now - self.start_time, jitter))
        self.last_loop_time = now

        # Define trajectory
        trajectory_points = [
            [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            [0.3, -0.3, 0.2, 0.1, 0.8, -0.1],
            [0.5,  0.0, -0.5, 0.5, 0.0, -0.5],
            [0.2,  0.2, -0.2, 0.2, -0.2, 0.2],
            [0.0,  0.0,  0.0, 0.0,  0.0, 0.0]
        ]

        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        for i, positions in enumerate(trajectory_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = (i + 1) * 2
            traj.points.append(point)

        traj.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(traj)
        self.get_logger().info('Published trajectory with 5 points.')

        # Stop after benchmark duration
        if time.time() - self.start_time >= self.max_duration:
            self.get_logger().info(f'Max duration reached ({self.max_duration}s). Saving jitter results...')
            self.save_results()
            rclpy.shutdown()

    def save_results(self):
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Loop Jitter (s)'])
            for row in self.jitter_log:
                writer.writerow(row)
        self.get_logger().info(f'Jitter data saved to {self.output_file}')

def main(args=None):
    rclpy.init(args=args)
    node = RTLoopMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

