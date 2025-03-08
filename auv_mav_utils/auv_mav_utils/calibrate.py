#!/usr/bin/env python3

import rclpy
import json
from rclpy.node import Node
from pymavlink import mavutil
from std_srvs.srv import Trigger


class DepthSensorCalibration(Node):
    """ROS2 node for calibrating depth sensor using MAV_CMD_PREFLIGHT_CALIBRATION."""

    def __init__(self):
        super().__init__("depth_sensor_calibration")

        # for OCU-> # self.connection_string = "udpout:0.0.0.0:14550"
        self.connection_string = "udp:0.0.0.0:14000"

        # Create MAVLink connection
        self.mav_connection = None

        # Create a service for calibration
        self.srv = self.create_service(
            Trigger, "calibrate_depth_sensor", self.calibrate_callback
        )

        self.get_logger().info("Depth Sensor Calibration Node started")
        self.get_logger().info(f"Connection string: {self.connection_string}")

    def setup_connection(self):
        """Establish MAVLink connection if not already connected."""
        if self.mav_connection is None:
            try:
                self.get_logger().info("Connecting to vehicle...")
                self.mav_connection = mavutil.mavlink_connection(self.connection_string)
                self.mav_connection.wait_heartbeat()
                self.get_logger().info(
                    f"Connected to system {self.mav_connection.target_system}"
                )
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to connect: {e}")
                return False
        return True

    def calibrate_callback(self, request, response):
        """Service callback to handle depth sensor calibration requests."""
        # Setup connection
        if self.mav_connection is None:
            self.setup_connection()

        try:
            # Create command message structure
            command_message = {
                "COMMAND_LONG": {
                    "message": {
                        "type": "common",
                        "type": "COMMAND_LONG",
                        "param1": 0.0,  # gyro cal
                        "param2": 0.0,  # mag cal
                        "param3": 1.0,  # ground pressure
                        "param4": 0.0,  # radio cal
                        "param5": 0.0,  # accel cal
                        "param6": 0.0,  # compass/motor interference
                        "param7": 0.0,  # depth sensor calibration
                        "command": {"type": "MAV_CMD_PREFLIGHT_CALIBRATION"},
                        "target_system": self.mav_connection.target_system,
                        "target_component": self.mav_connection.target_component,
                        "confirmation": 0,
                    }
                }
            }

            self.get_logger().info("Sending depth sensor calibration command...")
            self.get_logger().info(
                f"Command structure: {json.dumps(command_message, indent=2)}"
            )

            # Extract the actual command parameters from the structure
            cmd = command_message["COMMAND_LONG"]["message"]

            # Send calibration command using the structured format
            self.mav_connection.mav.command_long_send(
                cmd["target_system"],
                cmd["target_component"],
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                cmd["confirmation"],
                cmd["param1"],
                cmd["param2"],
                cmd["param3"],
                cmd["param4"],
                cmd["param5"],
                cmd["param6"],
                cmd["param7"],
            )

            # Wait for ACK
            ack_msg = self.mav_connection.recv_match(
                type="COMMAND_ACK", blocking=True, timeout=5
            )

            if (
                ack_msg
                and ack_msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
            ):
                if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Calibration command accepted")
                    response.success = True
                    response.message = "Depth sensor calibration started"
                else:
                    self.get_logger().error(
                        f"Command rejected with result: {ack_msg.result}"
                    )
                    response.success = False
                    response.message = (
                        f"Calibration command rejected (result: {ack_msg.result})"
                    )
            else:
                self.get_logger().warning("No ACK received for calibration command")
                response.success = False
                response.message = "No response from vehicle"

        except Exception as e:
            self.get_logger().error(f"Exception during calibration: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response


def main():
    """Main function to initialize and run the node."""
    rclpy.init()
    node = DepthSensorCalibration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
