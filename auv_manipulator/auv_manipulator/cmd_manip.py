import rclpy
from rclpy.node import Node
from auv_interfaces.msg import Manipulator


def get_user_input(prompt):
    try:
        return input(prompt)
    except EOFError:
        return ""


class CmdManip(Node):
    def __init__(self):
        super().__init__("cmd_manip")
        self.publisher_ = self.create_publisher(Manipulator, "manip_control", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Timer set to 500ms

    def timer_callback(self):
        message = Manipulator()
        message.arm_status = get_user_input("Arm Control (open/front/back): ")
        message.end_effector_status = get_user_input(
            "End Effector Control (close/open): "
        )

        self.get_logger().info(
            f"Publishing: \nArm Control-{message.arm_status} \nEnd Effector Control-{message.end_effector_status}"
        )
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CmdManip()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
