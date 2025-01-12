import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading


class VelocityPlotter(Node):
    def __init__(self):
        super().__init__("velocity_plotter")
        self.subscription = self.create_subscription(
            Vector3, "velocity", self.listener_callback, 10
        )
        self.x_data, self.y_data, self.z_data, self.time_data = [], [], [], []
        self.start_time = self.get_clock().now()

    def listener_callback(self, msg):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.time_data.append(current_time)
        self.x_data.append(msg.x)
        self.y_data.append(msg.y)
        self.z_data.append(msg.z)

    def get_data(self):
        return self.time_data, self.x_data, self.y_data, self.z_data


def plot_data(node):
    fig, ax = plt.subplots()
    ax.set_title("Velocity vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.grid()

    (line_x,) = ax.plot([], [], label="Velocity X", color="r")
    (line_y,) = ax.plot([], [], label="Velocity Y", color="g")
    (line_z,) = ax.plot([], [], label="Velocity Z", color="b")
    ax.legend()

    def update(frame):
        time_data, x_data, y_data, z_data = node.get_data()
        line_x.set_data(time_data, x_data)
        line_y.set_data(time_data, y_data)
        line_z.set_data(time_data, z_data)
        ax.relim()
        ax.autoscale_view()
        return line_x, line_y, line_z

    ani = FuncAnimation(fig, update, interval=100)
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
