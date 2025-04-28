#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Publish trajectory every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_trajectory)
        self.toggle = False  # To alternate positions

    def publish_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        point = JointTrajectoryPoint()

        if self.toggle:
            point.positions = [0.0, -0.5, 0.5, 0.0, 1.0, 0.0]
        else:
            point.positions = [0.5, 0.0, -0.5, 0.5, 0.0, -0.5]

        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0

        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info(f'Published trajectory: {point.positions}')

        # Switch for next loop
        self.toggle = not self.toggle

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

