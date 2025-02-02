import rclpy
from rclpy.node import Node
from auv_interfaces.srv import ManipulatorCommand


class ManipulatorClient(Node):
    def __init__(self):
        super().__init__("manipulator_client")
        self.client = self.create_client(ManipulatorCommand, "manipulator_command")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

    def send_command(self, arm_status, end_effector_status):
        request = ManipulatorCommand.Request()
        request.arm_status = arm_status
        request.end_effector_status = end_effector_status

        future = self.client.call_async(request)
        return future


def main():
    rclpy.init()
    client = ManipulatorClient()

    # Example command
    arm_status = int(input("Enter arm status (0/1): "))
    end_effector_status = int(input("Enter end effector status (0/1): "))
    future = client.send_command(arm_status, end_effector_status)

    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        response = future.result()
        client.get_logger().info(
            f"Service call completed with success: {response.success}, "
            f"message: {response.message}"
        )
    else:
        client.get_logger().error("Service call failed")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
