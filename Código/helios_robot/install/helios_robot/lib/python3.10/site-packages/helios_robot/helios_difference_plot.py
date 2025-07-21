#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class SensorDiffPlotter(Node):
    def __init__(self):
        super().__init__('sensor_diff_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_sensors_filtered',
            self.listener_callback,
            10)
        self.figs = {}  # Store figures for each group
        self.axes = {}  # Store axes for each group
        self.times = [[] for _ in range(4)]  # Time steps for each group
        self.x_values = [[] for _ in range(4)]  # X values for each group
        self.y_values = [[] for _ in range(4)]  # Y values for each group
        self.time_step = 0

        # Initialize 4 separate plot windows
        for i in range(4):
            self.figs[i], self.axes[i] = plt.subplots(figsize=(8, 6))
            plt.ion()  # Interactive mode

    def listener_callback(self, msg):
        data = msg.data
        if len(data) < 16:
            self.get_logger().warn('Received less than 16 values')
            return

        # Take first 16 values and divide into 4 groups of 4
        groups = [data[i:i+4] for i in range(0, 16, 4)]

        # Process each group
        for group_idx, group in enumerate(groups):
            if len(group) == 4:
                # Calculate X (1st - 3rd) and Y (2nd - 4th)
                x = group[0] - group[2]
                y = group[1] - group[3]

                # Store data
                self.times[group_idx].append(self.time_step)
                self.x_values[group_idx].append(x)
                self.y_values[group_idx].append(y)

                # Update plot for this group
                self.axes[group_idx].clear()
                self.axes[group_idx].plot(
                    self.times[group_idx], 
                    self.x_values[group_idx], 
                    'b-', 
                    label=f'X{group_idx+1} (1st - 3rd)'
                )
                self.axes[group_idx].plot(
                    self.times[group_idx], 
                    self.y_values[group_idx], 
                    'r-', 
                    label=f'Y{group_idx+1} (2nd - 4th)'
                )

                self.axes[group_idx].set_title(f'Group {group_idx+1} Differences')
                self.axes[group_idx].set_xlabel('Sample')
                self.axes[group_idx].set_ylabel('Difference')
                self.axes[group_idx].legend()
                self.axes[group_idx].grid(True)

                self.figs[group_idx].canvas.draw()
                self.figs[group_idx].canvas.flush_events()

                # Keep only last 100 points
                if len(self.times[group_idx]) > 100:
                    self.times[group_idx] = self.times[group_idx][-100:]
                    self.x_values[group_idx] = self.x_values[group_idx][-100:]
                    self.y_values[group_idx] = self.y_values[group_idx][-100:]

        self.time_step += 1
        plt.pause(0.001)

    def destroy_node(self):
        for fig in self.figs.values():
            plt.close(fig)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorDiffPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
