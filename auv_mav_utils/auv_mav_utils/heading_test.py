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
        self.movement_duration = self.get_parameter("movement_duration").value or 0.0
        self.angle_correction_enabled = False

        # Use a ReentrantCallbackGroup to allow for nested callbacks
        callback_group = ReentrantCallbackGroup()

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize action client
        self.depth_action_client = ActionClient(self, DepthDescent, "depth_descent")
        self.angle_correction_client = self.create_client(
            AngleCorrection, "angle_correction"
        )

        self.get_logger().info("Starting execution sequence")
        self.get_logger().info(f"Target depth: {self.target_depth}")
        self.get_logger().info(
            f'Angle correction: {"enabled" if self.angle_correction else "disabled"}'
        )
        self.get_logger().info(f"Linear speed: {self.linear_speed}")
        self.get_logger().info(f"Movement duration: {self.movement_duration} seconds")

        self.get_logger().info("Waiting for 10 seconds before starting depth action")
        time.sleep(10)  # Initial delay
        
        if self.enable_angle_correction:
                self.set_angle_correction(True)

        self.start_depth_descent()

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
        self.send_depth_action(0.025, surface_action_completed)

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

        # Send goal with feedback callback
        future = self.depth_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        # Wait for 3 seconds after depth action is completed before moving forward
        self.get_logger().info("Depth action completed. Waiting for 3 seconds before moving forward")
        time.sleep(3)
        self.move_forward()

    def move_forward(self):
        self.get_logger().info(
            f"Moving forward at {self.linear_speed} m/s for {self.movement_duration} seconds"
        )
        send_goal_future.add_done_callback(goal_response_callback)

        # Create timer to publish velocity commands
        self.movement_timer = self.create_timer(0.1, self.publish_velocity)
        
        # Create a timer to stop movement after the specified duration
        if self.movement_duration > 0:
            self.get_logger().info(f"Setting up stop timer for {self.movement_duration} seconds")
            self.stop_timer = self.create_timer(self.movement_duration, self.stop_movement)

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

        # Move forward for the specified duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist_message)
            time.sleep(0.05)  # Publish at 20Hz

    def stop_movement(self):
        self.get_logger().info("Stop timer triggered, stopping movement")
        
        # Cancel the stop timer to prevent multiple calls
        if hasattr(self, 'stop_timer'):
            self.stop_timer.cancel()
        
        # Cancel movement timer
        if hasattr(self, 'movement_timer'):
            self.movement_timer.cancel()

        # Stop the vehicle
        twist = Twist()
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Movement completed")
        
        # Wait for 3 seconds after movement is completed before surfacing
        self.get_logger().info("Waiting for 3 seconds before bringing up the vehicle")
        time.sleep(3)
        self.bring_up()

    def bring_up(self):
        self.get_logger().info("Bringing up the vehicle")
        
        # Wait for action server to be available
        if not self.depth_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Depth descent action server not available after timeout"
            )
            return

        # Create and send goal
        goal_msg = DepthDescent.Goal()
        goal_msg.target_depth = -0.25
        self.get_logger().info(f"Sending depth descent goal: {goal_msg.target_depth}")

        # Send goal with feedback callback
        future = self.depth_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)


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