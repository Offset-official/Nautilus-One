import rclpy
from rclpy.node import Node
from auv_interfaces.srv import ManipulatorCommand
from adafruit_servokit import ServoKit


class ManipController(Node):
    def __init__(self):
        super().__init__("manip_controller")

        self.kit = ServoKit(channels=16)

        self.kit.servo[0].angle = 0  # End Effector closed
        self.kit.servo[1].angle = 0  # Arm open

        self.service = self.create_service(
            ManipulatorCommand, "manipulator_command", self.command_callback
        )

        self.get_logger().info("Manipulator Controller Service has been started.")

    def command_callback(self, request, response):
        success = True
        messages = []

        # Control End Effector
        if request.end_effector_status.lower() == 0:
            self.kit.servo[0].angle = 0  # Closed position
            messages.append("End Effector set to CLOSED (0°)")
        elif request.end_effector_status.lower() == 1:
            self.kit.servo[0].angle = 110  # Open position
            messages.append("End Effector set to OPEN (110°)")
        else:
            success = False
            messages.append(
                f"Unknown End Effector Status: {request.end_effector_status}"
            )

        # Control Arm
        if request.arm_status.lower() == 1:
            self.kit.servo[1].angle = 0  # Open position
            messages.append("Arm set to OPEN (0°)")
        elif request.arm_status.lower() == 0:
            self.kit.servo[1].angle = 100  # Front position
            messages.append("Arm set to FRONT (100°)")
        else:
            success = False
            messages.append(f"Unknown Arm Status: {request.arm_status}")

        response.success = success
        response.message = "; ".join(messages)

        self.get_logger().info(f"Service call completed: {response.message}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManipController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "Manipulator Controller Service has been stopped manually."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
