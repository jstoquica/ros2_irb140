#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from rclpy.time import Time

class LatencyMeasurementNode(Node):
    def __init__(self):
        super().__init__('latency_measurement_node')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused var warning

    def listener_callback(self, msg):
        receive_time = self.get_clock().now()
        send_time = Time.from_msg(msg.header.stamp)

        latency_ns = (receive_time - send_time).nanoseconds
        latency_ms = latency_ns / 1e6
        self.get_logger().info(f'Latency: {latency_ms:.3f} ms')

def main(args=None):
    rclpy.init(args=args)
    node = LatencyMeasurementNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

