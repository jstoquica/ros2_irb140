#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import csv
import os

class TrajectoryBenchmarkLoop(Node):
    def __init__(self):
        super().__init__('trajectory_benchmark_loop')

        # Publisher to send joint trajectories
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Subscriber to the same topic to measure latency
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Timer to publish trajectory every 5 seconds
        self.timer_ = self.create_timer(5.0, self.publish_trajectory)

        # Benchmark parameters
        self.start_time = time.time()
        self.max_duration = 60  # seconds
        self.publish_log = []   # Records (elapsed, header.sec, header.nanosec, latency_ms)
        self.output_file = f'latency_log_{os.uname().nodename}.csv'

        # Logger
        self.get_logger().info('Trajectory communication benchmark with latency logging started.')

    def publish_trajectory(self):
        # Define 5 example trajectory points
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

        # Add current timestamp
        now = self.get_clock().now().to_msg()
        traj.header.stamp = now
        self.last_publish_stamp = now  # Store to compute latency in callback

        # Publish the message
        self.publisher_.publish(traj)
        self.get_logger().info('Published trajectory with 5 points.')

        # Check time to stop
        if (time.time() - self.start_time) >= self.max_duration:
            self.get_logger().info(f'Max duration reached. Saving latency log to {self.output_file}')
            self.save_results()
            rclpy.shutdown()

    def trajectory_callback(self, msg):
        """Callback to compute and log latency"""
        now = self.get_clock().now()
        sent = rclpy.time.Time.from_msg(msg.header.stamp)
        latency = (now - sent).nanoseconds / 1e6  # Convert to milliseconds
        elapsed = time.time() - self.start_time

        self.publish_log.append((
            round(elapsed, 3),
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            round(latency, 3)
        ))

        self.get_logger().info(f'Message latency: {latency:.3f} ms')

    def save_results(self):
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Elapsed Time (s)', 'ROS Timestamp Sec', 'ROS Timestamp Nsec', 'Latency (ms)'])
            for row in self.publish_log:
                writer.writerow(row)
        self.get_logger().info(f'Latency log saved to {self.output_file}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBenchmarkLoop()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

