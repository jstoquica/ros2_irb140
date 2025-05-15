#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
import psutil
import time
import csv
import os

class TrajectoryBenchmarkLoop(Node):
    def __init__(self):
        super().__init__('trajectory_benchmark_loop')

        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.ping_pub = self.create_publisher(Header, '/benchmark/ping', 10)
        self.pong_sub = self.create_subscription(Header, '/benchmark/pong', self.pong_callback, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        self.publish_timer = self.create_timer(5.0, self.publish_trajectory)
        self.ping_timer = self.create_timer(1.0, self.send_ping)
        self.log_timer = self.create_timer(1.0, self.log_metrics)

        self.start_time = time.time()
        self.max_duration = 60  # seconds
        self.sim_start = None
        self.sim_time = None
        self.latency_log = []
        self.output_file = 'benchmark_trajectory.csv'
        self.process = psutil.Process(os.getpid())

        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time (s)', 'CPU Time (s)', 'Memory (MiB)', 'Latency (ms)', 'Real-Time Factor'])

        self.get_logger().info('Trajectory benchmark loop node started.')

    def publish_trajectory(self):
        points = [
            [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            [0.3, -0.3, 0.2, 0.1, 0.8, -0.1],
            [0.5, 0.0, -0.5, 0.5, 0.0, -0.5],
            [0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]

        traj = JointTrajectory()
        traj.joint_names = [f'joint_{i+1}' for i in range(6)]

        for i, pos in enumerate(points):
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.time_from_start.sec = (i + 1) * 2
            pt.time_from_start.nanosec = 0
            traj.points.append(pt)

        traj.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(traj)
        self.get_logger().info('Published trajectory with 5 points.')

    def send_ping(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        self.ping_pub.publish(header)

    def pong_callback(self, msg):
        now = self.get_clock().now().to_msg()
        sent = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        received = now.sec + now.nanosec * 1e-9
        latency_ms = (received - sent) * 1000.0
        self.latency_log.append(latency_ms)

    def clock_callback(self, msg):
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_start is None:
            self.sim_start = sim_time
        self.sim_time = sim_time

    def log_metrics(self):
        elapsed = time.time() - self.start_time
        cpu_time = sum(self.process.cpu_times()[:2])  # user + system
        mem_mib = self.process.memory_info().rss / (1024 * 1024)
        latency = round(sum(self.latency_log) / len(self.latency_log), 3) if self.latency_log else 0.0
        self.latency_log.clear()

        rt_factor = round((self.sim_time - self.sim_start) / elapsed, 3) if self.sim_time and self.sim_start else 0.0

        self.get_logger().info(
            f"[{elapsed:.1f}s] CPU Time: {cpu_time:.2f}s, Mem: {mem_mib:.2f} MiB, Latency: {latency:.2f} ms, RT: {rt_factor:.2f}"
        )

        with open(self.output_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([round(elapsed, 2), round(cpu_time, 2), round(mem_mib, 2), latency, rt_factor])

        if elapsed >= self.max_duration:
            self.get_logger().info('Max duration reached. Shutting down...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBenchmarkLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

