#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
from numpy.linalg import norm, solve
import pinocchio
import tempfile


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.get_logger().info("Waiting for URDF on topic /robot_description...")

        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos_profile
        )
        self.robot = None

    def robot_description_callback(self, msg):
        self.get_logger().info("URDF received, building model...")
        urdf_string = msg.data

        with tempfile.NamedTemporaryFile('w+', delete=False, suffix='.urdf') as urdf_file:
            urdf_file.write(urdf_string)
            urdf_path = urdf_file.name

        try:
            free_flyer = pinocchio.JointModelFreeFlyer()
            model = pinocchio.buildModelFromUrdf(urdf_path, free_flyer)
            data = model.createData()
        except Exception as e:
            self.get_logger().error(f"Error building model: {e}")
            return

        JOINT_ID = 7  # Change this to the joint frame you want to target
        desired_pose = pinocchio.SE3(np.eye(3), np.array([1., 0., 1.]))
        q = pinocchio.neutral(model)
        eps = 1e-4
        max_iter = 1000
        dt = 0.1
        damp = 1e-12

        i = 0
        success = False
        while True:
            pinocchio.forwardKinematics(model, data, q)
            error_transform = desired_pose.actInv(data.oMi[JOINT_ID])
            error = pinocchio.log(error_transform).vector

            if norm(error) < eps:
                success = True
                break

            if i >= max_iter:
                break

            J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)
            v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), error))
            q = pinocchio.integrate(model, q, v * dt)

            if i % 10 == 0:
                self.get_logger().info(f'{i}: error = {error.T}')

            i += 1

        if success:
            self.get_logger().info("Convergence achieved!")
        else:
            self.get_logger().warn("Failed to converge to desired target.")

        self.get_logger().info(f'\nJoint angle q: {q.flatten().tolist()}')
        self.get_logger().info(f'\nJoint velocity v: {v.flatten().tolist()}')
        self.get_logger().info(f'\nFinal error: {error.T}')
        self.get_logger().info(f'Jacobian: {J}')
        self.get_logger().info(f'Joint name for ID {JOINT_ID}: {model.names[JOINT_ID]}')

        # Unsubscribe after processing once
        self.destroy_subscription(self.subscription)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
