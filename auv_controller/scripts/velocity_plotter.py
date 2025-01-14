#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading


class VelocityPlotter(Node):
    def __init__(self):
        super().__init__("velocity_plotter")
        self.subscription_imu = self.create_subscription(
            Vector3, "imu_vel", self.listener_callback_imu, 10
        )
        self.subscription_control = self.create_subscription(
            Vector3, "applied_vel", self.listener_callback_control, 10
        )
        self.subscription_ref = self.create_subscription(
            Twist, "cmd_vel", self.listener_callback_ref, 10
        )
        self.time_data = []
        self.imu_data = {"x": [], "y": [], "z": []}
        self.control_data = {"x": [], "y": [], "z": []}
        self.ref_data = {"x": [], "y": [], "z": []}
        self.start_time = self.get_clock().now()

    def listener_callback_imu(self, msg):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if not self.time_data or current_time > self.time_data[-1]:  # Avoid duplicates
            self.time_data.append(current_time)
        self.imu_data["x"].append(msg.x)
        self.imu_data["y"].append(msg.y)
        self.imu_data["z"].append(msg.z)

    def listener_callback_control(self, msg):
        self.control_data["x"].append(msg.x)
        self.control_data["y"].append(msg.y)
        self.control_data["z"].append(msg.z)

    def listener_callback_ref(self, msg):
        print(msg)
        self.ref_data["x"].append(msg.linear.y)
        self.ref_data["y"].append(msg.angular.z)
        self.ref_data["z"].append(msg.linear.z)


    def get_data(self):
        min_length = min(
            len(self.time_data),
            len(self.imu_data["x"]),
            len(self.control_data["x"]),
            len(self.ref_data["x"]),
        )
        return (
            self.time_data[:min_length],
            {k: v[:min_length] for k, v in self.imu_data.items()},
            {k: v[:min_length] for k, v in self.control_data.items()},
            {k: v[:min_length] for k, v in self.ref_data.items()},
        )


def plot_data(node):
    fig, axs = plt.subplots(3, 1, figsize=(8, 12))
    titles = ["Surge", "Yaw ", "Heave"]
    colors = ["r", "g", "b"]

    lines_imu = []
    lines_control = []
    lines_ref = []

    for ax, title, color in zip(axs, titles, colors):
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.grid()
        (line_imu,) = ax.plot([], [], label="IMU", color=color)
        (line_control,) = ax.plot(
            [], [], label="Control", linestyle="--", color="orange"
        )
        (line_ref,) = ax.plot([], [], label="Reference", linestyle=":", color="blue")
        lines_imu.append(line_imu)
        lines_control.append(line_control)
        lines_ref.append(line_ref)
        ax.legend()

    def update(frame):
        time_data, imu_data, control_data, ref_data = node.get_data()
        for i, key in enumerate(["x", "y", "z"]):
            lines_imu[i].set_data(time_data, imu_data[key])
            lines_control[i].set_data(time_data, control_data[key])
            lines_ref[i].set_data(time_data, ref_data[key])
            axs[i].relim()
            axs[i].autoscale_view()
        return lines_imu + lines_control + lines_ref

    ani = FuncAnimation(fig, update, interval=100)
    plt.tight_layout()
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    plotter_node = VelocityPlotter()
    threading.Thread(target=rclpy.spin, args=(plotter_node,), daemon=True).start()
    plot_data(plotter_node)
    plotter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
