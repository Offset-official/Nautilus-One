#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from auv_interfaces.action import DepthDescent
from auv_interfaces.srv import AngleCorrection


class HeadingTest(Node):
    def __init__(self):
        super().__init__("heading_test")

        # Initialize member variables
        self.target_depth = -0.6  # Default value
        self.angle_correction = True  # Default value
        self.linear_speed = 1.0  # Default value
        self.movement_duration = 10.0  # Default 10 seconds

        # Declare parameters
        self.declare_parameter("target_depth", self.target_depth)
        self.declare_parameter("angle_correction", self.angle_correction)
        self.declare_parameter("linear_speed", self.linear_speed)
        self.declare_parameter("movement_duration", self.movement_duration)

        # Get parameters
        self.target_depth = self.get_parameter("target_depth").value
        self.angle_correction = self.get_parameter("angle_correction").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.movement_duration = self.get_parameter("movement_duration").value

        # Use a ReentrantCallbackGroup to allow for nested callbacks
        callback_group = ReentrantCallbackGroup()

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize action client
        self.depth_action_client = ActionClient(
            self, DepthDescent, "depth_descent", callback_group=callback_group
        )

        # Initialize service client
        self.angle_correction_client = self.create_client(
            AngleCorrection, "angle_correction", callback_group=callback_group
        )

        # Start the execution sequence in a separate thread
        sequence_thread = threading.Thread(target=self.execute_sequence)
        sequence_thread.daemon = True
        sequence_thread.start()

    def execute_sequence(self):
        # Wait a moment for everything to initialize
        time.sleep(1.0)

        self.get_logger().info("Starting execution sequence")
        self.get_logger().info(f"Target depth: {self.target_depth}")
        self.get_logger().info(
            f'Angle correction: {"enabled" if self.angle_correction else "disabled"}'
        )
        self.get_logger().info(f"Linear speed: {self.linear_speed}")
        self.get_logger().info(f"Movement duration: {self.movement_duration} seconds")

        # Step 1: Enable angle correction if needed
        if self.angle_correction:
            self.get_logger().info("Step 1: Enabling angle correction")
            angle_correction_success = self.set_angle_correction(True)

            if not angle_correction_success:
                self.get_logger().warn(
                    "Failed to enable angle correction, continuing with sequence anyway"
                )

        # Step 2: Wait for 10 seconds
        self.get_logger().info("Step 2: Waiting for 10 seconds")
        time.sleep(10.0)

        # Step 3: Send depth action
        self.get_logger().info(
            f"Step 3: Starting descent to target depth: {self.target_depth}"
        )
        depth_action_completed = threading.Event()
        self.send_depth_action(self.target_depth, depth_action_completed)

        # Wait for depth action to complete
        depth_action_completed.wait()

        # Step 4: Wait for 5 seconds after depth descent
        self.get_logger().info("Step 4: Waiting for 5 seconds after depth descent")
        time.sleep(5.0)

        # Step 5: Move forward for specified duration
        self.get_logger().info(
            f"Step 5: Moving forward with speed {self.linear_speed} for {self.movement_duration} seconds"
        )
        self.perform_forward_movement()

        # Step 6: Wait for 5 seconds after forward motion
        self.get_logger().info("Step 6: Waiting for 5 seconds after forward motion")
        time.sleep(5.0)

        # Step 7: Return to surface
        self.get_logger().info("Step 7: Returning to surface (depth 0.025)")
        surface_action_completed = threading.Event()
        self.send_depth_action(-0.1, surface_action_completed)

        # Wait for return to surface to complete
        surface_action_completed.wait()

        # Step 8: Final log message
        self.get_logger().info("Sequence completed successfully!")

    def send_depth_action(self, target_depth, completion_event):
        # Wait for action server
        if not self.depth_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Depth descent action server not available after waiting 5 seconds"
            )
            completion_event.set()  # Prevent deadlock
            return

        # Create goal message
        goal_msg = DepthDescent.Goal()
        goal_msg.target_depth = target_depth

        self.get_logger().info(f"Sending depth descent goal: {target_depth}")

        # Define callbacks
        def feedback_callback(feedback_msg):
            self.get_logger().info(
                f"Depth descent feedback - Current depth: {feedback_msg.feedback.current_depth}"
            )

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by server")
                completion_event.set()  # Prevent deadlock
                return

            self.get_logger().info("Goal accepted by server")

            # Request result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def result_callback(future):
            result = future.result().result
            self.get_logger().info(
                f"Depth action completed with final depth: {result.final_depth}"
            )
            completion_event.set()

        # Send goal
        send_goal_future = self.depth_action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )
        send_goal_future.add_done_callback(goal_response_callback)

    def perform_forward_movement(self):
        # Create twist message for forward movement
        twist_message = Twist()
        twist_message.linear.y = float(self.linear_speed)

        # Start time tracking
        start_time = time.time()
        end_time = start_time + float(self.movement_duration)

        # Move forward for the specified duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist_message)
            time.sleep(0.05)  # Publish at 20Hz

        # Stop movement
        stop_message = Twist()
        stop_message.linear.y = 0.0

        # Publish stop command multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_message)
            time.sleep(0.05)

        self.get_logger().info("Forward movement completed")

    def set_angle_correction(self, enable):
        # Wait for service to be available
        if not self.angle_correction_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "Angle correction service not available after waiting 5 seconds"
            )
            return False

        # Create request
        request = AngleCorrection.Request()
        request.enable = enable

        # Send request
        future = self.angle_correction_client.call_async(request)

        # Wait for response in this thread (blocking)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Successfully {"enabled" if enable else "disabled"} angle correction'
                )
                return True
            else:
                self.get_logger().error(
                    f'Failed to {"enable" if enable else "disable"} angle correction'
                )
                return False
        else:
            self.get_logger().error("Service call timed out")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = HeadingTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()