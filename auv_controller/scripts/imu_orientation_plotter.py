#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math


class OrientationPlotter(Node):
    def __init__(self):
        super().__init__("orientation_plotter")

        # Define QoS profile for IMU data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to IMU data
        self.subscription_imu = self.create_subscription(
            Imu, "mavros/imu/data", self.listener_callback_imu, sensor_qos
        )

        self.time_data = []
        self.orientation_data = {"roll": [], "pitch": [], "yaw": []}
        self.start_time = self.get_clock().now()

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback_imu(self, msg):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.time_data.append(current_time)
        self.orientation_data["roll"].append(roll_deg)
        self.orientation_data["pitch"].append(pitch_deg)
        self.orientation_data["yaw"].append(yaw_deg)

    def get_data(self):
        return self.time_data, self.orientation_data


def plot_data(node):
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))
    titles = ["Roll", "Pitch", "Yaw"]
    colors = ["r", "g", "b"]
    lines = []

    for ax, title, color in zip(axs, titles, colors):
        ax.set_title(f"{title} Angle")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (degrees)")
        ax.grid(True)
        (line,) = ax.plot([], [], label=title, color=color)
        lines.append(line)
        ax.legend()

    def update(frame):
        time_data, orientation_data = node.get_data()
        if not time_data:  # No data yet
            return lines

        for i, angle in enumerate(["roll", "pitch", "yaw"]):
            lines[i].set_data(time_data, orientation_data[angle])
            axs[i].relim()
            axs[i].set_ylim(-60, 60)  # Set y-axis limits to ±180 degrees
            axs[i].autoscale_view(scalex=True, scaley=False)

        fig.canvas.draw_idle()
        return lines

    ani = FuncAnimation(fig, update, interval=100)
    plt.tight_layout()
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    plotter_node = OrientationPlotter()
    threading.Thread(target=rclpy.spin, args=(plotter_node,), daemon=True).start()
    plot_data(plotter_node)
    plotter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
