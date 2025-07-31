import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

class URDFSaverNode(Node):
    def __init__(self):
        super().__init__("urdf_saver_node")
        self.get_logger().info("Waiting for URDF on topic /robot_description...")

        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.subscription = self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile
        )

    def urdf_callback(self, msg):
        self.get_logger().info("URDF received. Saving to disk...")

        urdf_string = msg.data

        try:
            # Define path to save the URDF
            home_dir = os.path.expanduser("~")
            save_path = os.path.join(home_dir, "my_robot")
            os.makedirs(save_path, exist_ok=True)

            urdf_filename = os.path.join(save_path, "robot.urdf")

            # Save the URDF file
            with open(urdf_filename, "w") as f:
                f.write(urdf_string)

            self.get_logger().info(f"URDF saved to: {urdf_filename}")

            # Unsubscribe to stop saving more URDFs
            self.destroy_subscription(self.subscription)
            self.get_logger().info("Subscription destroyed, will not save more URDFs.")

        except Exception as e:
            self.get_logger().error(f"Failed to save URDF: {e}")

def main(args=None):
    import rclpy
    rclpy.init(args=args)

    node = URDFSaverNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
