#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Force TkAgg backend
import matplotlib.pyplot as plt
import os
import time

class SensorPlotNode(Node):
    def __init__(self):
        super().__init__('sensor_plot_node')
        # Check display environment
        self.has_display = bool(os.environ.get('DISPLAY'))
        self.get_logger().info(f'Display available: {self.has_display}')

        # Subscriber to raw sensor data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_sensors_filtered',
            self.sensor_callback,
            10)
        self.get_logger().info('Subscribed to /helios_sensors_filtered')

        # Store sensor history for Pieces 1-4 (4 pieces, each with 4 sensors)
        self.num_pieces = 4
        self.sensor_history = [[[] for _ in range(4)] for _ in range(self.num_pieces)]  # [piece][sensor]
        self.time_steps = []
        self.time_step = 0
        # Maximum number of time steps to display
        self.max_time_steps = 100
        # Plot output directory
        self.plot_dir = 'sensor_plots'
        os.makedirs(self.plot_dir, exist_ok=True)
        # Timeout for no data (seconds)
        self.timeout = 10.0
        self.last_data_time = time.time()

        # Setup plots for each piece
        self.figs = []
        self.axes = []
        self.lines = [[] for _ in range(self.num_pieces)]
        self.directions = ['North', 'East', 'South', 'West']
        colors = ['b', 'g', 'r', 'm']  # Blue, Green, Red, Magenta for Pieces 1-4
        linestyles = ['-', '--', ':', '-.']  # Styles for each sensor

        for piece in range(self.num_pieces):
            fig, ax = plt.subplots(1, 1, figsize=(10, 6))
            self.figs.append(fig)
            self.axes.append(ax)
            piece_lines = []
            for sensor in range(4):
                label = f'Piece {piece + 1} {self.directions[sensor]}'
                line, = ax.plot([], [], 
                               color=colors[piece], 
                               linestyle=linestyles[sensor], 
                               label=label)
                piece_lines.append(line)
            self.lines[piece] = piece_lines
            ax.set_title(f'Piece {piece + 1} Sensor Values')
            ax.set_xlabel('Time Step')
            ax.set_ylabel('Sensor Value')
            ax.grid(True)
            ax.legend()
            fig.tight_layout()

            self.get_logger().info(f'Initialized plot for Piece {piece + 1}')

        # Show all figures if display is available
        if self.has_display:
            plt.show(block=False)

        # Timer to update plots (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.update_plots)

    def sensor_callback(self, msg):
        # Extract sensor data (expect 24 values, use indices 0-15 for Pieces 1-4)
        sensor_data = np.array(msg.data, dtype=np.float32)
        if len(sensor_data) < 16:
            self.get_logger().warn('Received less than 16 values')
            return
        sensor_data = sensor_data[0:16]  # Pieces 1-4: indices 0 to 15

        self.get_logger().info(f'Received sensor data: {sensor_data.tolist()}')
        self.last_data_time = time.time()

        # Update sensor history for each piece
        for piece in range(self.num_pieces):
            piece_data = sensor_data[piece * 4:(piece + 1) * 4]  # 4 values per piece
            for sensor in range(4):
                self.sensor_history[piece][sensor].append(piece_data[sensor])
        self.time_step += 1
        self.time_steps.append(self.time_step)

        # Trim history to max_time_steps
        if len(self.time_steps) > self.max_time_steps:
            self.time_steps.pop(0)
            for piece in range(self.num_pieces):
                for sensor in range(4):
                    self.sensor_history[piece][sensor].pop(0)

    def update_plots(self):
        # Check for timeout
        if time.time() - self.last_data_time > self.timeout:
            self.get_logger().error('No data received for 10 seconds. Shutting down.')
            for fig in self.figs:
                plt.close(fig)
            rclpy.shutdown()
            return

        self.get_logger().info('Updating plots')
        # Update plots for each piece
        for piece in range(self.num_pieces):
            # Update lines for this piece
            for sensor_idx in range(4):
                self.lines[piece][sensor_idx].set_data(
                    self.time_steps,
                    self.sensor_history[piece][sensor_idx]
                )
            # Update axis limits
            if self.time_steps:
                self.axes[piece].set_xlim(max(0, self.time_steps[0]), max(self.time_steps[-1], 1))
                y_data = [y for sensor_hist in self.sensor_history[piece] for y in sensor_hist if not np.isnan(y)]
                if y_data:
                    y_min, y_max = min(y_data), max(y_data)
                    y_range = y_max - y_min
                    if y_range == 0:
                        y_range = 0.1
                    self.axes[piece].set_ylim(y_min - 0.1 * y_range, y_max + 0.1 * y_range)

            # Redraw plot
            if self.has_display:
                self.figs[piece].canvas.draw()
                self.figs[piece].canvas.flush_events()

            # Save plot
            plot_path = os.path.join(self.plot_dir, f'sensor_plot_piece_{piece + 1}_{self.time_step}.png')
            try:
                self.figs[piece].savefig(plot_path)
                self.get_logger().info(f'Saved plot to {plot_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save plot for Piece {piece + 1}: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down node')
        # Save final plots
        for piece in range(self.num_pieces):
            plot_path = os.path.join(self.plot_dir, f'sensor_plot_piece_{piece + 1}_final.png')
            try:
                self.figs[piece].savefig(plot_path)
                self.get_logger().info(f'Saved final plot to {plot_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save final plot for Piece {piece + 1}: {e}')
            plt.close(self.figs[piece])
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SensorPlotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
