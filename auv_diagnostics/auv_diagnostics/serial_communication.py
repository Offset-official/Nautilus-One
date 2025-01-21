import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

BAUD_RATE = 9600

class SerialCommunication(Node):
    def __init__(self):
        super().__init__('serial_communication_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').get_parameter_value().string_value

        # Initialize serial communication
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.5)
            time.sleep(1)
            self.get_logger().info('Serial communication initialized')
        except serial.SerialException:
            self.get_logger().error('Failed to initialize serial communication')

        # Subscriber for /diagnostics
        self.subscription = self.create_subscription(
            String,
            '/diagnostics',
            self.serial_callback,
            10
        )

    # Send diagnostics message to serial port
    def serial_callback(self, msg):
        self.ser.write(msg.data.encode('utf-8') + b'\n')
        self.get_logger().info(f"Sent message: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunication()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

