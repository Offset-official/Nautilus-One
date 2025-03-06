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
    """Node for testing heading control of an autonomous underwater vehicle."""

    def __init__(self):
        super().__init__("heading_test")
        self._initialize_parameters()
        self._initialize_publishers()
        self._initialize_clients()
        self._log_startup_info()
        self._execute_mission()

    def _initialize_parameters(self):
        """Initialize and load all parameters."""
        # Declare parameters with default values
        self.declare_parameter("target_depth", -0.6)
        self.declare_parameter("enable_angle_correction", True)
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("movement_duration", 10.0)

        # Load parameters
        self.target_depth = self.get_parameter("target_depth").value
        self.enable_angle_correction = self.get_parameter(
            "enable_angle_correction"
        ).value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.movement_duration = self.get_parameter("movement_duration").value

        # Additional state variables
        self.angle_correction_enabled = False
        self.move_timer = None
        self.stop_timer = None

    def _initialize_publishers(self):
        """Initialize all publishers."""
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _initialize_clients(self):
        """Initialize all action and service clients."""
        self.depth_action_client = ActionClient(self, DepthDescent, "depth_descent")
        self.angle_correction_client = self.create_client(
            AngleCorrection, "angle_correction"
        )

    def _log_startup_info(self):
        """Log the startup configuration."""
        self.get_logger().info("Starting execution sequence")
        self.get_logger().info(f"Target depth: {self.target_depth}")
        self.get_logger().info(
            f'Angle correction: {"enabled" if self.enable_angle_correction else "disabled"}'
        )
        self.get_logger().info(f"Linear speed: {self.linear_speed}")
        self.get_logger().info(f"Movement duration: {self.movement_duration} seconds")

    def _execute_mission(self):
        """Execute the main mission sequence."""
        self.get_logger().info("Waiting for 10 seconds before starting depth action")
        time.sleep(10)  # Initial delay
        self.send_depth_action(self.target_depth)

    def send_depth_action(self, target_depth):
        """Send the depth descent action goal."""
        if not self.depth_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Depth descent action server not available")
            return

        goal_msg = DepthDescent.Goal()
        goal_msg.target_depth = target_depth

        self.get_logger().info(f"Sending depth descent goal: {target_depth}")

        self.depth_action_client.send_goal_async(
            goal_msg, feedback_callback=self._depth_feedback_callback
        ).add_done_callback(self._depth_goal_response_callback)

    def _depth_feedback_callback(self, feedback_msg):
        """Process feedback from the depth descent action."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Depth descent feedback - Current depth: {feedback.current_depth}"
        )

    def _depth_goal_response_callback(self, future):
        """Handle the response to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server")
            return

        self.get_logger().info("Goal accepted by server")
        goal_handle.get_result_async().add_done_callback(self._depth_result_callback)

    def _depth_result_callback(self, future):
        """Process the depth action result and trigger next phase."""
        try:
            result = future.result().result
            status = future.result().status
            
            self.get_logger().info(f"Depth action completed with status: {status}")

            if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Depth action succeeded. Starting next phase.")

                if self.enable_angle_correction:
                    self.set_angle_correction(True)

                # Use a timer instead of sleep to avoid blocking
                self.create_timer(10.0, self.start_forward_movement)
            else:
                self.get_logger().error(f"Depth action did not succeed, status: {status}")
        except Exception as e:
            self.get_logger().error(f"Exception in depth result callback: {str(e)}")

    def start_forward_movement(self):
        """Begin forward movement phase."""
        twist_message = Twist()
        twist_message.linear.y = self.linear_speed

        self.get_logger().info(
            f"Moving forward with speed: {self.linear_speed} for {self.movement_duration} seconds"
        )

        # Publish velocity command continuously
        self.move_timer = self.create_timer(
            0.05, lambda: self.cmd_vel_pub.publish(twist_message)
        )

        # Create a timer to stop movement after the specified duration
        self.stop_timer = self.create_timer(self.movement_duration, self._stop_movement)

    def _stop_movement(self):
        """Stop the forward movement and clean up timers."""
        # Stop movement
        stop_message = Twist()
        self.cmd_vel_pub.publish(stop_message)  # All velocities default to 0.0

        self.get_logger().info(
            f"Stopped moving forward after {self.movement_duration} seconds."
        )

        # Cancel the timers
        if self.move_timer:
            self.move_timer.cancel()
            self.move_timer = None

        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        # Create a new timer for the completion sequence
        # This separates the stopping from the completion sequence
        self.create_timer(0.1, self._complete_mission_sequence)

    def _complete_mission_sequence(self):
        """Complete the mission sequence with angle correction disable and surface."""
        # Cancel this one-shot timer
        for timer in self.get_timers():
            if timer.callback == self._complete_mission_sequence:
                timer.cancel()
                break

        self.get_logger().info("Completing mission sequence")

        # Disable angle correction
        self.set_angle_correction(False)

        # Wait for angle correction to be disabled
        time.sleep(5)

        # Final action - come to surface
        # This should be the last command with no functions after it
        self.get_logger().info("Mission complete, returning to surface")
        self.send_surface_action()

    def send_surface_action(self):
        """Send the final depth action to return to surface."""
        if not self.depth_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                "Depth descent action server not available for surface action"
            )
            return

        goal_msg = DepthDescent.Goal()
        goal_msg.target_depth = -0.05  # Surface depth

        self.get_logger().info("Sending final command: return to surface")

        # For the final action, we don't add any callback that would execute additional code
        self.depth_action_client.send_goal_async(goal_msg)

    def set_angle_correction(self, enable):
        """Enable or disable angle correction."""
        if not self.angle_correction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Angle correction service not available")
            return

        request = AngleCorrection.Request()
        request.enable = enable

        future = self.angle_correction_client.call_async(request)
        future.add_done_callback(
            partial(self._angle_correction_callback, enable=enable)
        )

    def _angle_correction_callback(self, future, enable):
        """Handle the response from the angle correction service."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f'Successfully {"enabled" if enable else "disabled"} angle correction'
                )
                self.angle_correction_enabled = enable
            else:
                self.get_logger().error(
                    f'Failed to {"enable" if enable else "disable"} angle correction'
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = HeadingTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        if node.move_timer:
            node.move_timer.cancel()
        if node.stop_timer:
            node.stop_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
