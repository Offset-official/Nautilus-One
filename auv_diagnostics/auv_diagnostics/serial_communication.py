import rclpy
from rclpy.node import Node
import serial
import time
import json
from std_msgs.msg import String

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

class SerialCommunication(Node):
    def __init__(self):
        super().__init__('serial_communication_node')

        # Initialize serial communication
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
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
        data = json.dumps(msg.data)
        self.ser.write(data.encode('utf-8') + b'\n')
        self.get_logger().info('Sent diagnostics message to serial port')

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

