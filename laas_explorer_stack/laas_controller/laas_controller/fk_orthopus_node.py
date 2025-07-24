#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import pinocchio as pin
import tempfile


class URDFPinocchioNode(Node):
    def __init__(self):
        super().__init__("urdf_pinocchio_node")
        self.get_logger().info("Waiting for URDF on topic /robot_description...")

        # Create QoS with transient local durability to receive already published messages
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile
        )

    def urdf_callback(self, msg):
        self.get_logger().info("URDF received. Saving and loading with Pinocchio...")

        urdf_string = msg.data

        try:
            # Save the URDF to a temporary file
            with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix=".urdf") as temp_file:
                temp_file.write(urdf_string)
                urdf_filename = temp_file.name

            self.get_logger().info(f"URDF saved to: {urdf_filename}")

            # Load the model with Pinocchio using free-flyer base
            model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
            data = model.createData()

            self.get_logger().info(f"Pinocchio model loaded: {model.name}")

            # Run forward kinematics with a random configuration
            q = pin.randomConfiguration(model)
            pin.forwardKinematics(model, data, q)

            # Print the position of each joint frame
            for name, oMi in zip(model.names, data.oMi):
                print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

            self.get_logger().info("Forward kinematics completed successfully.")
            # Optionally unsubscribe to stop processing more messages
            self.destroy_subscription(self.subscription)

        except Exception as e:
            self.get_logger().error(f"Failed to load URDF or compute kinematics: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = URDFPinocchioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
