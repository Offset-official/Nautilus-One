#!/usr/bin/env python3

import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from auv_interfaces.action import DepthDescent
from auv_interfaces.srv import AngleCorrection


class HeadingTest(Node):
    def __init__(self):
        super().__init__("heading_test")
        self.declare_parameter("target_depth", -0.6)
        self.declare_parameter("enable_angle_correction", True)
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("movement_duration", 10.0)

        self.target_depth = self.get_parameter("target_depth").value
        self.enable_angle_correction = self.get_parameter(
            "enable_angle_correction"
        ).value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.movement_duration = self.get_parameter("movement_duration").value

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize action client
        self.depth_action_client = ActionClient(self, DepthDescent, "depth_descent")

        self.get_logger().info("Starting execution sequence")
        self.get_logger().info(f"Target depth: {self.target_depth}")
        self.get_logger().info(
            f'Angle correction: {"enabled" if self.enable_angle_correction else "disabled"}'
        )
        self.get_logger().info(f"Linear speed: {self.linear_speed}")
        self.get_logger().info(f"Movement duration: {self.movement_duration} seconds")

        self.get_logger().info("Waiting for 10 seconds before starting depth action")
        time.sleep(10)  # Initial delay
        self.start_depth_descent()
        time.sleep(10)
        self.move_forward()

    def start_depth_descent(self):
        # Wait for action server to be available
        if not self.depth_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Depth descent action server not available after timeout"
            )
            return

        # Create and send goal
        goal_msg = DepthDescent.Goal()
        goal_msg.target_depth = self.target_depth
        self.get_logger().info(f"Sending depth descent goal: {self.target_depth}")

        # Send goal with feedback callback
        self._send_goal_future = self.depth_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self._logger.info("Goal completed")
        result = future.result().result
        self.get_logger().info(
            f"Depth descent completed. Final depth: {result.final_depth}"
        )

        # After depth descent is complete, proceed with angle correction if enabled
        if self.enable_angle_correction:
            self.get_logger().info("Starting angle correction...")
            self.correct_angle()
        else:
            self.get_logger().info(
                "Angle correction disabled, proceeding with forward movement"
            )

    def feedback_callback(self, feedback_msg):
        # Process feedback from action server
        current_depth = feedback_msg.feedback.current_depth
        self.get_logger().info(f"Current depth: {current_depth}")

    def correct_angle(self):
        # This would be implemented to call the angle correction service
        # For now, just proceed to forward movement
        self.get_logger().info("Angle correction would be performed here")
        self.move_forward()

    def move_forward(self):
        self.get_logger().info(
            f"Moving forward at {self.linear_speed} m/s for {self.movement_duration} seconds"
        )

        # Create timer to publish velocity commands
        self.movement_timer = self.create_timer(0.1, self.publish_velocity)

        # Create timer to stop movement after duration
        self.stop_timer = self.create_timer(self.movement_duration, self.stop_movement)

    def publish_velocity(self):
        # Create and publish forward velocity command
        twist = Twist()
        twist.linear.y = self.linear_speed
        self.cmd_vel_pub.publish(twist)

    def stop_movement(self):
        # Stop the vehicle
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Movement completed")

        # Cancel timers
        self.movement_timer.cancel()
        self.stop_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    heading_test_node = HeadingTest()

    try:
        rclpy.spin(heading_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        heading_test_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
