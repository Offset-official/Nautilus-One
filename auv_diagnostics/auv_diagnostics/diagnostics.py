import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
import serial
import json
import time
from std_msgs.msg import String

class AUVDiagnostics(Node):
    def __init__(self):
        super().__init__('auv_diagnostics_node')

        # Subscriber for /mavros/state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Publisher for /diagnostics
        self.publisher = self.create_publisher(
            String,
            '/diagnostics',
            10
        )

        self.states_str = ""


    # Log diagnostics message
    def state_callback(self, msg):
        states_json = {
            "armed": msg.armed,
        }

        # Serialize JSON to string
        self.states_str = json.dumps(states_json)

        self.get_logger().info(self.states_str)

    # Publish diagnostics message
    def publish_callback(self, msg):
        data = self.states_str.encode('utf-8')
        self.publisher.publish(data)
        

def main(args=None):
    rclpy.init(args=args)
    node = AUVDiagnostics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
