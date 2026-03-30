#!/usr/bin/env python3
"""
Record OpenVINS estimate and EuRoC ground truth to text files
for use with ov_eval error_singlerun.

Output format (space-separated):
  timestamp(s) tx ty tz qx qy qz qw

Usage:
  python3 record_poses.py
  (run while ros2 bag play is active, Ctrl+C when done)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped


class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')

        self.f_est = open('state_estimate.txt', 'w')
        self.f_gt  = open('state_groundtruth.txt', 'w')
        self.f_est.write('# timestamp(s) tx ty tz qx qy qz qw\n')
        self.f_gt.write( '# timestamp(s) tx ty tz qx qy qz qw\n')

        self.sub_est = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ov_msckf/poseimu',
            self.cb_est,
            100)

        self.sub_gt = self.create_subscription(
            TransformStamped,
            '/vicon/firefly_sbx/firefly_sbx',
            self.cb_gt,
            100)

        self.get_logger().info('Recording /ov_msckf/poseimu -> state_estimate.txt')
        self.get_logger().info('Recording /vicon/firefly_sbx/firefly_sbx -> state_groundtruth.txt')
        self.get_logger().info('Press Ctrl+C when the bag finishes.')

    def cb_est(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f_est.write(f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')

    def cb_gt(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.transform.translation
        q = msg.transform.rotation
        self.f_gt.write(f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')

    def destroy_node(self):
        self.f_est.close()
        self.f_gt.close()
        self.get_logger().info('Files saved: state_estimate.txt, state_groundtruth.txt')
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
