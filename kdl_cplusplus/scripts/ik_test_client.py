#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_kinematics.srv import ComputeIK


class IKClient(Node):
    def __init__(self):
        super().__init__('ik_test_client')
        self.cli = self.create_client(ComputeIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

    def call(self):
        req = ComputeIK.Request()
        req.joint_names = ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6']
        req.seed_positions = [0.0] * 6
        req.target.header.frame_id = 'base_link'
        req.target.pose.position.x = 0.30
        req.target.pose.position.y = 0.10
        req.target.pose.position.z = 0.25
        req.target.pose.orientation.w = 1.0
        req.base_link = 'base_link'
        req.tip_link = 'End-Coupler-v1'
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = IKClient()
    res = node.call()
    print('success:', res.success)
    print('message:', res.message)
    print('solution_positions:', list(res.solution_positions))
    rclpy.shutdown()


if __name__ == '__main__':
    main()


