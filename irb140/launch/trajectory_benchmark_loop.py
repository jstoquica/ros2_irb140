#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.clock import Clock, ClockType

class TrajectoryBenchmarkNode(Node):
    def __init__(self):
        super().__init__('trajectory_benchmark_node')

        # Joint names must match the controller exactly
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        # Trajectory definition
        self.trajectory_points = [
            [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            [0.3, -0.3, 0.2, 0.1, 0.8, -0.1],
            [0.5,  0.0, -0.5, 0.5, 0.0, -0.5],
            [0.2,  0.2, -0.2, 0.2, -0.2, 0.2],
            [0.0,  0.0,  0.0, 0.0,  0.0, 0.0]
        ]

        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

        # Latency measurement
        self.latency_list = []
        self.latency_sample_size = 20
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('TrajectoryBenchmarkNode started.')

    def publish_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        time_from_start = 0.0
        for positions in self.trajectory_points:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
            traj.points.append(point)
            time_from_start += 2.0  # 2 seconds between each point

        traj.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(traj)
        self.get_logger().info('Trajectory published.')

    def joint_state_callback(self, msg):
        try:
            msg_time_ros = Time.from_msg(msg.header.stamp)
            now = Clock(clock_type=ClockType.ROS_TIME).now()
            latency = (now - msg_time_ros).nanoseconds / 1e6  # ms
            self.latency_list.append(latency)

            if len(self.latency_list) >= self.latency_sample_size:
                avg_latency = sum(self.latency_list) / len(self.latency_list)
                self.get_logger().info(f'[Latency] Avg joint_states latency: {avg_latency:.3f} ms over {self.latency_sample_size} samples')
                self.latency_list.clear()
        except Exception as e:
            self.get_logger().error(f'Error in latency calculation: {e}')

def main():
    rclpy.init()
    node = TrajectoryBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

