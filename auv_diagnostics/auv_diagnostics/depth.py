import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymavlink import mavutil


class MavlinkDepthPublisher(Node):
    def __init__(self):
        super().__init__("mavlink_depth_publisher")

        # Create a publisher on the /depth topic
        self.publisher_ = self.create_publisher(Float32, "/depth", 10)

        # Connect to MAVLink (adjust for your connection type)
        self.connection = mavutil.mavlink_connection("udp:0.0.0.0:14550")  # SITL
        # self.connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)  # Serial

        # Wait for MAVLink heartbeat
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to MAVLink vehicle")

        # Start the loop to receive and publish data
        self.timer = self.create_timer(0.005, self.get_ahrs_data)  # Runs every 100ms

    def get_ahrs_data(self):
        """Fetch AHRS message and publish altitude."""
        msg = self.connection.recv_match(type="AHRS2", blocking=False)
        if msg:
            altitude = msg.altitude  # Extract altitude from AHRS message
            self.get_logger().info(f"Publishing Depth: {altitude:.2f} meters")

            # Publish to /depth topic
            depth_msg = Float32()
            depth_msg.data = altitude
            self.publisher_.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    mavlink_depth_publisher = MavlinkDepthPublisher()
    rclpy.spin(mavlink_depth_publisher)
    mavlink_depth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
