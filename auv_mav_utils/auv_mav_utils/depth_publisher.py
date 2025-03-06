import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pymavlink import mavutil


# mavproxy.py --out udp:127.0.0.1:5760 --master udp:0.0.0.0:14550 --console
class MavlinkDepthPublisher(Node):
    def __init__(self):
        super().__init__("mavlink_depth_publisher")

        # Create a publisher on the /depth topic
        self.publisher_ = self.create_publisher(Float64, "/current_depth", 5)

        # Connect to MAVLink (adjust for your connection type)
        self.connection = mavutil.mavlink_connection("udp:0.0.0.0:14000")  # SITL
        # self.connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)  # Serial

        # Wait for MAVLink heartbeat
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to MAVLink vehicle")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,  # The MAVLink message ID
            1e6 / 100,  # The interval between two messages in microseconds.
            0,
            0,
            0,
            0,  # Unused parameters
            0,
        )
        # Start the loop to receive and publish data
        self.timer = self.create_timer(0.001, self.get_ahrs_data)  # Runs every 100ms

    def get_ahrs_data(self):
        """Fetch AHRS message and publish altitude."""
        msg = self.connection.recv_match(type="AHRS2", blocking=False)
        if msg:
            altitude = msg.altitude  # Extract altitude from AHRS message
            # self.get_logger().info(f"Publishing Depth: {altitude:.2f} meters")

            # Publish to /depth topic
            depth_msg = Float64()
            depth_msg.data = altitude
            self.publisher_.publish(depth_msg)


def main(args=None):
    rclpy.init()
    mavlink_depth_publisher = MavlinkDepthPublisher()
    rclpy.spin(mavlink_depth_publisher)
    mavlink_depth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
