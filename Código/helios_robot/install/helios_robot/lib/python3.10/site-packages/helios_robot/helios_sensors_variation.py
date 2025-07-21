import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import matplotlib.pyplot as plt
import numpy as np
import array

class SensorVariationPlotter(Node):
    def __init__(self):
        super().__init__('sensor_variation_plotter')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_sensors_filtered',
            self.listener_callback,
            10)
        
        # Create publishers for each piece's difference
        self.piece_publishers = [
            self.create_publisher(Float32, 'piece_1_difference', 10),
            self.create_publisher(Float32, 'piece_2_difference', 10),
            self.create_publisher(Float32, 'piece_3_difference', 10),
            self.create_publisher(Float32, 'piece_4_difference', 10)
        ]
        
        # Initialize variables for plotting
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 8))
        self.fig.suptitle('Sensor Data Variations (4-value Windows)')
        
        # Initialize data storage for each window
        self.window_variations = [[] for _ in range(4)]
        self.timestamps = [[] for _ in range(4)]
        
        # Previous values for calculating variations
        self.prev_values = None
        
        # Counter for timestamps
        self.counter = 0
        
        # Set up plots for each window
        titles = [
            'Sum of Variations - Values 0-3 (Piece 1)',
            'Sum of Variations - Values 4-7 (Piece 2)',
            'Sum of Variations - Values 8-11 (Piece 3)',
            'Sum of Variations - Values 12-15 (Piece 4)'
        ]
        
        colors = ['b', 'g', 'r', 'm']
        
        for i, ax in enumerate(self.axes):
            ax.set_title(titles[i])
            ax.set_xlabel('Time')
            ax.set_ylabel('Variation Sum')
            ax.grid(True)
            # Initialize empty plot with color
            ax.plot([], [], color=colors[i], label=titles[i])
            ax.legend()
        
        plt.tight_layout()
        plt.ion()  # Interactive mode on
        plt.show()

    def listener_callback(self, msg):
        # Convert array.array to list for easier handling
        current_values = list(msg.data)
        
        # Check if we have previous values to compare with and enough data
        if self.prev_values is not None and len(current_values) >= 16:
            # Process each window of 4 values
            for window_idx in range(4):
                start_idx = window_idx * 4
                end_idx = start_idx + 4
                
                # Calculate variation sum for this window
                window_variation = 0.0
                for i in range(start_idx, end_idx):
                    window_variation += abs(current_values[i] - self.prev_values[i])
                
                # Store the variation for plotting
                self.window_variations[window_idx].append(window_variation)
                self.timestamps[window_idx].append(self.counter)
                
                # Publish the variation for this piece
                pub_msg = Float32()
                pub_msg.data = window_variation
                self.piece_publishers[window_idx].publish(pub_msg)
            
            # Update plots
            self.update_plots()
            
            self.counter += 1
        
        # Store current values for next comparison
        self.prev_values = current_values

    def update_plots(self):
        # Update each subplot
        for i, ax in enumerate(self.axes):
            ax.clear()
            ax.plot(self.timestamps[i], self.window_variations[i], 
                   color=self.get_color(i), 
                   label=f'Piece {i+1} (Values {i*4}-{i*4+3})')
            ax.set_title(f'Sum of Variations - Piece {i+1} (Values {i*4}-{i*4+3})')
            ax.set_xlabel('Time')
            ax.set_ylabel('Variation Sum')
            ax.grid(True)
            ax.legend()
        
        # Adjust layout and draw
        self.fig.tight_layout()
        plt.draw()
        plt.pause(0.001)
    
    def get_color(self, index):
        """Return a consistent color for each window"""
        colors = ['blue', 'green', 'red', 'purple']
        return colors[index]

def main(args=None):
    rclpy.init(args=args)
    sensor_variation_plotter = SensorVariationPlotter()
    rclpy.spin(sensor_variation_plotter)
    
    # Cleanup
    sensor_variation_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
