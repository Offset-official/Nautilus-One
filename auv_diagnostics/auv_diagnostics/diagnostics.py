import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState 
import json
from std_msgs.msg import String
from auv_interfaces.srv import SetColor


class AUVDiagnostics(Node):
    def __init__(self):
        super().__init__('auv_diagnostics_node')

        # Subscribers
        self.create_subscription(
            State,
            '/mavros/state',
            self.navigator_state_callback,
            10
        )

        self.create_subscription(
            BatteryState,  # Correct message type
            '/mavros/battery',
            self.battery_nav_callback,
            10
        )

        # Services
        self.set_color_service = self.create_service(
            SetColor,
            '/set_color',
            self.set_color_callback
        )

        # Publishers
        self.publisher = self.create_publisher(
            String,
            '/diagnostics',
            10
        )

        # Initializing state variables with defaults
        self.navigator_state_data = {
            "armed": False,
        }
        self.battery_nav_data = {
            "percentage": "NA",
        }

        self.color = "off" 
        self.color_count = 0
        self.timer = self.create_timer(0.5, self.publish_diagnostics)

    # Callback for /mavros/state
    def navigator_state_callback(self, msg):
        self.navigator_state_data["armed"] = msg.armed

    # Callback for /mavros/battery
    def battery_nav_callback(self, msg):
        self.battery_nav_data["percentage"] = msg.percentage if hasattr(msg, "percentage") else "NA"

    # Callback for /set_color service
    def set_color_callback(self, request, response):
        if request.color.lower() == "off" or request.color.startswith("#"):
            self.color = request.color
            self.color_count = request.color_count
            response.success = True
            response.message = f"Color set to {request.color}"
        else:
            response.success = False
            response.message = "Invalid color"

        return response
    


    def publish_diagnostics(self):
        # Combine state and battery data
        diagnostics = {
            "armed": self.navigator_state_data["armed"],
            "batteryN": "NA",
            "batteryJ": "NA", 
            "task": "QLFN",
            "jetson_connection": False,
            "neopixel_color": self.color,
            "neopixel_count": self.color_count    
        }

        # Convert to JSON and publish
        msg = String()
        msg.data = json.dumps(diagnostics)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published diagnostics: {msg.data}")


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
