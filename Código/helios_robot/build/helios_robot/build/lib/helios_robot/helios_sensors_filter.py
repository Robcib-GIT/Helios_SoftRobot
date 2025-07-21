#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class SensorFilterNode(Node):
    def __init__(self):
        super().__init__('sensor_filter_node')
        # Subscriber to raw sensor data on /helios_sensors
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_sensors',
            self.sensor_callback,
            10)
        # Publisher for filtered sensor data on /helios_sensors_filtered
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/helios_sensors_filtered',
            10)

        # Number of values per message (must match publisher)
        self.num_channels = 24

        # Initialize history for median filter
        self.raw_history = np.full((3, self.num_channels), np.nan, dtype=np.float32)

        self.get_logger().info("Median filter initialized with window size 3")

    def sensor_callback(self, msg: Float32MultiArray):
        # Convert incoming data list to NumPy array of float32
        sensor_data = np.array(msg.data, dtype=np.float32)

        # Sanity check: ensure correct number of channels
        if sensor_data.size != self.num_channels:
            self.get_logger().warn(
                f'Expected {self.num_channels} values, received {sensor_data.size}')
            return

        # Shift history
        self.raw_history[0, :] = self.raw_history[1, :]
        self.raw_history[1, :] = self.raw_history[2, :]
        self.raw_history[2, :] = sensor_data

        # Compute median for each channel, ignoring NaN
        filtered_data = np.nanmedian(self.raw_history, axis=0).astype(np.float32)

        # Publish filtered_data as Float32MultiArray
        filtered_msg = Float32MultiArray()
        filtered_msg.data = filtered_data.tolist()
        self.publisher.publish(filtered_msg)

        self.get_logger().debug(f'Published median-filtered data: {filtered_data.tolist()}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
