#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import psutil
import time
import csv
import os

class TrajectoryBenchmarkLoop(Node):
    def __init__(self):
        super().__init__('trajectory_benchmark_loop')

        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer_ = self.create_timer(5.0, self.publish_trajectory)

        self.cpu_log = []
        self.start_time = time.time()
        self.max_duration = 60  # seconds
        self.output_file = f'benchmark_{os.uname().nodename}.csv'

        self.logger = self.get_logger()
        self.logger.info('Benchmark node started on KR260...')

    def publish_trajectory(self):
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
            point.time_from_start.nanosec = 0
            traj.points.append(point)

        now = self.get_clock().now().to_msg()
        traj.header.stamp = now  # Send time

        self.publisher_.publish(traj)
        self.get_logger().info(f'Published trajectory at {now.sec}.{now.nanosec:09d}')

        self.log_system_usage()

        if time.time() - self.start_time >= self.max_duration:
            self.get_logger().info(f'Max duration reached ({self.max_duration}s). Saving results...')
            self.save_results()
            rclpy.shutdown()

    def log_system_usage(self):
        elapsed = time.time() - self.start_time
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        self.cpu_log.append((elapsed, cpu, mem))

    def save_results(self):
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'CPU Usage (%)', 'Memory Usage (%)'])
            for row in self.cpu_log:
                writer.writerow(row)
        self.get_logger().info(f'Saved system metrics to {self.output_file}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBenchmarkLoop()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

