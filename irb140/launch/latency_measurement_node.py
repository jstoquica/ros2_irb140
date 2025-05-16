#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from rclpy.time import Time
import csv
import os
from datetime import datetime

class LatencyMeasurementNode(Node):
    def __init__(self):
        super().__init__('latency_measurement_node')

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback,
            10)

        # Prepare output CSV file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        hostname = os.uname().nodename
        self.output_file = f'latency_log_{hostname}_{timestamp}.csv'

        self.csv_file = open(self.output_file, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Send Time (s)', 'Receive Time (s)', 'Latency (ms)'])

        self.get_logger().info(f'Latency logger started. Saving to {self.output_file}')

    def listener_callback(self, msg):
        receive_time = self.get_clock().now()
        send_time = Time.from_msg(msg.header.stamp)

        latency_ns = (receive_time - send_time).nanoseconds
        latency_ms = latency_ns / 1e6

        send_time_float = send_time.nanoseconds / 1e9
        receive_time_float = receive_time.nanoseconds / 1e9

        self.csv_writer.writerow([f'{send_time_float:.9f}', f'{receive_time_float:.9f}', f'{latency_ms:.3f}'])
        self.get_logger().info(f'Latency: {latency_ms:.3f} ms')

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'CSV file saved: {self.output_file}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LatencyMeasurementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

