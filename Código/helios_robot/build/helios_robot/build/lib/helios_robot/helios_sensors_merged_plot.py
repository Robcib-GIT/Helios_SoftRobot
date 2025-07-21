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

        # Store sensor history for merged pieces (2 merged pieces, each with 4 sensors)
        self.num_merged_pieces = 2
        self.sensor_history = [[[] for _ in range(4)] for _ in range(self.num_merged_pieces)]  # [merged_piece][sensor]
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

        # Setup plots for each merged piece
        self.figs = []
        self.axes = []
        self.lines = [[] for _ in range(self.num_merged_pieces)]
        self.directions = ['North', 'East', 'South', 'West']
        colors = ['b', 'r']  # Blue for merged 1-2, Red for merged 3-4
        linestyles = ['-', '--', ':', '-.']  # Styles for each sensor

        for piece in range(self.num_merged_pieces):
            fig, ax = plt.subplots(1, 1, figsize=(10, 6))
            self.figs.append(fig)
            self.axes.append(ax)
            piece_lines = []
            for sensor in range(4):
                if piece == 0:
                    label = f'Merged 1-2 {self.directions[sensor]}'
                else:
                    label = f'Merged 3-4 {self.directions[sensor]}'
                line, = ax.plot([], [], 
                               color=colors[piece], 
                               linestyle=linestyles[sensor], 
                               label=label)
                piece_lines.append(line)
            self.lines[piece] = piece_lines
            ax.set_title(f'Merged Pieces {piece*2+1}-{piece*2+2} Sensor Values')
            ax.set_xlabel('Time Step')
            ax.set_ylabel('Sensor Value')
            ax.grid(True)
            ax.legend()
            fig.tight_layout()

            self.get_logger().info(f'Initialized plot for Merged Pieces {piece*2+1}-{piece*2+2}')

        # Show all figures if display is available
        if self.has_display:
            plt.show(block=False)

        # Timer to update plots (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.update_plots)

    def sensor_callback(self, msg):
        # Extract sensor data (expect at least 16 values for Pieces 1-4)
        sensor_data = np.array(msg.data, dtype=np.float32)
        if len(sensor_data) < 16:
            self.get_logger().warn('Received less than 16 values')
            return
        sensor_data = sensor_data[0:16]  # Use only the first 16 values

        self.get_logger().info(f'Received sensor data: {sensor_data.tolist()}')
        self.last_data_time = time.time()

        # Compute merged sensor values
        # Merged piece 1-2
        merged_1_2_N = sensor_data[0] + sensor_data[4]
        merged_1_2_E = sensor_data[1] + sensor_data[5]
        merged_1_2_S = sensor_data[2] + sensor_data[6]
        merged_1_2_W = sensor_data[3] + sensor_data[7]
        # Merged piece 3-4
        merged_3_4_N = sensor_data[8] + sensor_data[12]
        merged_3_4_E = sensor_data[9] + sensor_data[13]
        merged_3_4_S = sensor_data[10] + sensor_data[14]
        merged_3_4_W = sensor_data[11] + sensor_data[15]

        # Update sensor history for each merged piece
        # Merged piece 1-2
        self.sensor_history[0][0].append(merged_1_2_N)
        self.sensor_history[0][1].append(merged_1_2_E)
        self.sensor_history[0][2].append(merged_1_2_S)
        self.sensor_history[0][3].append(merged_1_2_W)
        # Merged piece 3-4
        self.sensor_history[1][0].append(merged_3_4_N)
        self.sensor_history[1][1].append(merged_3_4_E)
        self.sensor_history[1][2].append(merged_3_4_S)
        self.sensor_history[1][3].append(merged_3_4_W)

        self.time_step += 1
        self.time_steps.append(self.time_step)

        # Trim history to max_time_steps
        if len(self.time_steps) > self.max_time_steps:
            self.time_steps.pop(0)
            for piece in range(self.num_merged_pieces):
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
        # Update plots for each merged piece
        for piece in range(self.num_merged_pieces):
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
            plot_path = os.path.join(self.plot_dir, f'sensor_plot_merged_{piece+1}_{self.time_step}.png')
            try:
                self.figs[piece].savefig(plot_path)
                self.get_logger().info(f'Saved plot to {plot_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save plot for Merged Pieces {piece*2+1}-{piece*2+2}: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down node')
        # Save final plots
        for piece in range(self.num_merged_pieces):
            plot_path = os.path.join(self.plot_dir, f'sensor_plot_merged_{piece+1}_final.png')
            try:
                self.figs[piece].savefig(plot_path)
                self.get_logger().info(f'Saved final plot to {plot_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save final plot for Merged Pieces {piece*2+1}-{piece*2+2}: {e}')
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
