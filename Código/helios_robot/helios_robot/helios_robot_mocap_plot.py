import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')
        # Initialize data storage and threading lock
        self.data_lock = threading.Lock()
        self.base_pose = None
        self.triangle_pose = None
        self.meas_array = None

        # Create subscribers
        self.sub_base = self.create_subscription(
            Pose, 'HeliosBase/pose', self.base_callback, 10)
        self.sub_triangle = self.create_subscription(
            Pose, 'HeliosTriangle/pose', self.triangle_callback, 10)
        self.sub_meas = self.create_subscription(
            Float32MultiArray, 'helios_pose_meas', self.meas_callback, 10)

    def base_callback(self, msg):
        with self.data_lock:
            self.base_pose = msg

    def triangle_callback(self, msg):
        with self.data_lock:
            self.triangle_pose = msg

    def meas_callback(self, msg):
        with self.data_lock:
            self.meas_array = msg.data

def main():
    rclpy.init()
    node = PlotNode()

    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    # Set up matplotlib plot
    fig, ax = plt.subplots()
    ax.set_xlim([-10, 10])  # Adjust limits based on expected data range
    ax.set_ylim([-10, 10])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Dynamic Pose and Measurement Plot')
    scatter_rot = ax.scatter([], [], color='red', label='Rotated Difference')
    scatter_meas = ax.scatter([], [], color='blue', label='Measurement')
    ax.legend()

    def update_plot(frame):
        with node.data_lock:
            # Check if all data is available
            if (node.base_pose is None or 
                node.triangle_pose is None or 
                node.meas_array is None or 
                len(node.meas_array) < 2):
                return scatter_rot, scatter_meas

            # Compute position difference
            diff_x = node.triangle_pose.position.x - node.base_pose.position.x
            diff_y = node.triangle_pose.position.y - node.base_pose.position.y
            diff_z = node.triangle_pose.position.z - node.base_pose.position.z

            # Apply rotation around z-axis (alpha = pi/2)
            alpha = np.pi / 2
            x_rot = diff_x * np.cos(alpha) - diff_y * np.sin(alpha)
            y_rot = diff_x * np.sin(alpha) + diff_y * np.cos(alpha)

            # Get first measurement pair
            x_meas = node.meas_array[0]
            y_meas = node.meas_array[1]

            # Update scatter plots
            scatter_rot.set_offsets([[x_rot, y_rot]])
            scatter_meas.set_offsets([[x_meas, y_meas]])

        return scatter_rot, scatter_meas

    # Set up animation (update every 100ms)
    ani = FuncAnimation(fig, update_plot, interval=100, blit=True)
    plt.show()

    # Clean up when plot window is closed
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
