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

        # ROS 2 publisher for joint trajectories
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Timer to publish trajectory every 5 seconds
        self.timer_ = self.create_timer(5.0, self.publish_trajectory)

        # Benchmark tracking variables
        self.cpu_log = []
        self.mem_log = []
        self.start_time = time.time()
        self.max_duration = 60  # Total benchmark duration in seconds
        self.output_file = f'benchmark_{os.uname().nodename}.csv'

        # Logger for output
        self.logger = self.get_logger()
        self.logger.info('Benchmark node started...')

    def publish_trajectory(self):
        # Define multiple trajectory points (5 in total)
        trajectory_points = [
            [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            [0.3, -0.3, 0.2, 0.1, 0.8, -0.1],
            [0.5,  0.0, -0.5, 0.5, 0.0, -0.5],
            [0.2,  0.2, -0.2, 0.2, -0.2, 0.2],
            [0.0,  0.0,  0.0, 0.0,  0.0, 0.0]
        ]

        # Create the trajectory message
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # Add each point with increasing time_from_start
        for i, positions in enumerate(trajectory_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = (i + 1) * 2  # each point 2 seconds apart
            point.time_from_start.nanosec = 0
            traj.points.append(point)

        # Set the timestamp of the trajectory
        traj.header.stamp = self.get_clock().now().to_msg()

        # Publish the trajectory
        self.publisher_.publish(traj)
        self.get_logger().info('Published trajectory with 5 points.')

        # Log system performance metrics
        self.log_system_usage()

        # Stop the benchmark after max_duration
        elapsed = time.time() - self.start_time
        if elapsed >= self.max_duration:
            self.get_logger().info(f'Max duration reached ({self.max_duration}s). Saving results...')
            self.save_results()
            rclpy.shutdown()

    def log_system_usage(self):
        """Log CPU and memory usage for benchmarking"""
        elapsed = time.time() - self.start_time
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        self.cpu_log.append((elapsed, cpu, mem))

    def save_results(self):
        """Save benchmarking results to a CSV file"""
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'CPU Usage (%)', 'Memory Usage (%)'])
            for row in self.cpu_log:
                writer.writerow(row)
        self.get_logger().info(f'Benchmark data saved to {self.output_file}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBenchmarkLoop()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

