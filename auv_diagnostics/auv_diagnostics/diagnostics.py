import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
import json
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

        self.timer = self.create_timer(2, self.publish_diagnostics)


    # Log diagnostics message
    def state_callback(self, msg):
        states_json = {
            "armed": msg.armed ,
        }
        # store as JSON string
        self.states_str = json.dumps(states_json)
        self.get_logger().info(f"Received state: {self.states_str}")

    def publish_diagnostics(self):
        # create a String message
        msg = String()
        msg.data = self.states_str
        self.publisher.publish(msg)
        

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
