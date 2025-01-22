import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

BAUD_RATE = 115200 # Baud rate for serial communication

class SerialCommunication(Node):
    def __init__(self):
        super().__init__('serial_communication_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').get_parameter_value().string_value

        # Initialize serial communication
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=2.0) 
            time.sleep(1)
            self.get_logger().info('Serial communication initialized')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to initialize serial communication: {e}')

        # Subscriber for /diagnostics
        self.subscription = self.create_subscription(
            String,
            '/diagnostics',
            self.serial_callback,
            10
        )

    def serial_callback(self, msg):
        try:
            self.get_logger().info(f"Message size: {len(msg.data)} bytes")
            self.ser.write(msg.data.encode('utf-8') + b'\n')
            self.ser.flush()
            time.sleep(0.1)  
            self.get_logger().info(f"Sent message: {msg.data}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.handle_serial_disconnection()

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

    def handle_serial_disconnection(self):
        try:
            self.ser.close()
        except Exception as e:
            self.get_logger().warn(f"Error while closing serial port: {e}")

        self.get_logger().info("Attempting to reconnect to serial port...")
        available_ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
        for port in available_ports:
            try:
                self.ser.port = port  
                self.ser.open()      
                self.get_logger().info(f"Reconnected to serial port: {port}")
                return
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to {port}: {e}")

        self.get_logger().error("Reconnection attempts to all ports failed.")



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
