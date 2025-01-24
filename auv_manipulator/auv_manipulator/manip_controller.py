import rclpy
from rclpy.node import Node
from auv_interfaces.msg import Manipulator
from adafruit_servokit import ServoKit


class ManipController(Node):
    def __init__(self):
        super().__init__("manip_controller")

        # Initialize the ServoKit with 16 channels
        self.kit = ServoKit(channels=16)

        # Initialize servos to default positions
        self.kit.servo[0].angle = 0  # End Effector closed
        self.kit.servo[1].angle = 0  # Arm open

        # Create a subscription to the "manip_control" topic
        self.subscription = self.create_subscription(
            Manipulator, "manip_control", self.topic_callback, 10
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info("Manipulator Controller Node has been started.")

    def topic_callback(self, msg):
        self.get_logger().info(
            f"Received Control Command:\n"
            f"Arm Status: {msg.arm_status}\n"
            f"End Effector Status: {msg.end_effector_status}"
        )

        # Control End Effector
        if msg.end_effector_status.lower() == "close":
            self.kit.servo[0].angle = 0  # Closed position
            self.get_logger().info("End Effector set to CLOSED (0°).")
        elif msg.end_effector_status.lower() == "open":
            self.kit.servo[0].angle = 110  # Open position
            self.get_logger().info("End Effector set to OPEN (110°).")
        else:
            self.get_logger().warn(
                f"Unknown End Effector Status: {msg.end_effector_status}. "
                "No action taken."
            )

        # Control Arm
        if msg.arm_status.lower() == "open":
            self.kit.servo[1].angle = 0  # Open position
            self.get_logger().info("Arm set to OPEN (0°).")
        elif msg.arm_status.lower() == "front":
            self.kit.servo[1].angle = 100  # Front position
            self.get_logger().info("Arm set to FRONT (100°).")
        else:
            self.get_logger().warn(
                f"Unknown Arm Status: {msg.arm_status}. " "No action taken."
            )


def main(args=None):
    rclpy.init(args=args)
    node = ManipController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Manipulator Controller Node has been stopped manually.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
