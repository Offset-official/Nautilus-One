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
        self.subscription_ref = self.create_subscription(
            Twist, "cmd_vel", self.listener_callback_ref, 10
        )
        self.time_data = []
        self.imu_data = {"x": [], "y": []}
        self.ref_data = {"x": [], "y": []}
        self.start_time = self.get_clock().now()

        # Initialize last known values
        self.last_imu = {"x": 0.0, "y": 0.0}
        self.last_ref = {"x": 0.0, "y": 0.0}

    def listener_callback_imu(self, msg):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if not self.time_data or current_time > self.time_data[-1]:  # Avoid duplicates
            self.time_data.append(current_time)
            # Update last known values
            self.last_imu["x"] = msg.x
            self.last_imu["y"] = msg.y
            # Append to data lists
            self.imu_data["x"].append(msg.x)
            self.imu_data["y"].append(msg.y)
            # Use last known values for other datasets if they haven't been updated
            self._append_last_values(current_time)

    def listener_callback_ref(self, msg):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.last_ref["x"] = msg.linear.y
        self.last_ref["y"] = msg.angular.z
        if not self.time_data or current_time > self.time_data[-1]:
            self.time_data.append(current_time)
            self._append_last_values(current_time)

    def _append_last_values(self, current_time):
        # Ensure all data lists have the same length as time_data
        target_length = len(self.time_data)

        for data_list, last_values in [
            (self.imu_data, self.last_imu),
            (self.ref_data, self.last_ref),
        ]:
            for axis in ["x", "y"]:
                while len(data_list[axis]) < target_length:
                    data_list[axis].append(last_values[axis])

    def get_data(self):
        return (self.time_data, self.imu_data, self.ref_data)


def plot_data(node):
    fig, axs = plt.subplots(2, 1, figsize=(8, 8))
    titles = ["Surge", "Yaw"]
    colors = ["r", "g"]

    lines_imu = []
    lines_ref = []

    for ax, title, color in zip(axs, titles, colors):
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.grid()
        (line_imu,) = ax.plot([], [], label="IMU", color=color)
        (line_ref,) = ax.plot([], [], label="Reference", linestyle=":", color="orange")
        lines_imu.append(line_imu)
        lines_ref.append(line_ref)
        ax.legend()

    def update(frame):
        time_data, imu_data, ref_data = node.get_data()
        for i, key in enumerate(["x", "y"]):
            lines_imu[i].set_data(time_data, imu_data[key])
            lines_ref[i].set_data(time_data, ref_data[key])
            axs[i].relim()
            axs[i].set_ylim(bottom=-2.0, top=2.0)
            axs[i].autoscale_view(scalex=True, scaley=True)
        fig.canvas.draw_idle()
        return lines_imu + lines_ref

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
