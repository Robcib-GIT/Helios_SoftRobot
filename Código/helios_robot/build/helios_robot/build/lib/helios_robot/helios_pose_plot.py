import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_helios_xy_node')
        # Initialize data storage and threading lock
        self.data_lock = threading.Lock()
        self.x_values = []
        self.y_values = []
        
        # Create subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_pose_meas',
            self.meas_callback,
            10)
        
        # Set up matplotlib plot
        self.fig, self.ax = plt.subplots()
        self.scatter, = self.ax.plot([], [], 'bo', label='x (first), y (seventh)')
        self.ax.set_xlabel('X (First Value)')
        self.ax.set_ylabel('Y (Seventh Value)')
        self.ax.set_title('Dynamic Plot of /helios_pose_meas (x, y)')
        self.ax.legend()
        
        # Set up animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)

    def meas_callback(self, msg):
        # Extract first and seventh values if available
        if len(msg.data) >= 7:
            with self.data_lock:
                self.x_values.append(msg.data[0])
                self.y_values.append(msg.data[6])

    def update_plot(self, frame):
        with self.data_lock:
            if self.x_values and self.y_values:
                self.scatter.set_data(self.x_values, self.y_values)
                self.ax.relim()
                self.ax.autoscale_view()
        return self.scatter,

def main():
    rclpy.init()
    node = PlotNode()
    
    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    # Show plot
    plt.show()
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
