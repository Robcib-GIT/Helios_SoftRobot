import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class CableController(Node):
    def __init__(self):
        super().__init__('cable_controller')
        
        # Create subscribers for piece differences
        self.piece1_sub = self.create_subscription(
            Float32, '/piece_1_difference', self.piece1_callback, 10)
        
        self.piece3_sub = self.create_subscription(
            Float32, '/piece_3_difference', self.piece3_callback, 10)
        
        # Create publisher for cable commands
        self.cmd_publisher = self.create_publisher(
            Float32MultiArray, '/helios_cables_cmd', 10)
        
        # Initialize state variables
        self.piece1_diff = 0.0
        self.piece3_diff = 0.0
        self.current_cable = 0  # Tracks which cable we're testing (0-3)
        self.testing_active = True
        self.waiting_between_cables = False
        self.wait_start_time = None
        self.wait_duration = 2.0  # Wait 2 seconds between cables
        
        # Create a timer to publish commands at 5Hz (slower frequency)
        self.timer = self.create_timer(1, self.timer_callback)  # 5Hz
        
        self.get_logger().info("Cable controller started. Beginning cable testing sequence.")
    
    def piece1_callback(self, msg):
        self.piece1_diff = msg.data
    
    def piece3_callback(self, msg):
        self.piece3_diff = msg.data
    
    def timer_callback(self):
        if not self.testing_active:
            # If testing is complete, publish zeros
            cmd = [0.0] * 12
            self.publish_command(cmd)
            return
        
        # Check if we're waiting between cables
        if self.waiting_between_cables:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.wait_start_time).nanoseconds / 1e9
            
            if elapsed_time >= self.wait_duration:
                # Wait period finished, start testing next cable
                self.waiting_between_cables = False
                self.get_logger().info(f"Starting test for cable {self.current_cable}")
            else:
                # Still waiting, publish zeros
                cmd = [0.0] * 12
                self.publish_command(cmd)
                return
        
        # Create base command with all zeros
        cmd = [0.0] * 12
        
        # Check if piece1_difference is below threshold (0.007)
        if self.piece1_diff < 0.007:
            # Set current cable command: negative for even indices (0, 2), positive for odd (1, 3)
            cmd[self.current_cable] = -0.001 if self.current_cable % 2 == 0 else 0.001
            self.publish_command(cmd)
        else:
            # Threshold reached for current cable - immediately set all cables to zero
            self.get_logger().info(
                f"Cable {self.current_cable} threshold reached. "
                f"(Piece1: {self.piece1_diff:.4f}) - Setting all cables to zero"
            )
            
            # First, publish zeros for all cables
            cmd = [0.0] * 12
            self.publish_command(cmd)
            
            # Move to next cable
            self.current_cable += 1
            
            # Check if we've tested all cables
            if self.current_cable >= 4:
                self.get_logger().info("All 4 cables tested. Stopping sequence.")
                self.testing_active = False
            else:
                # Start waiting period before next cable
                self.waiting_between_cables = True
                self.wait_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting {self.wait_duration} seconds before testing cable {self.current_cable}")
            
            # Reset differences for next test
            self.piece1_diff = 0.0
            self.piece3_diff = 0.0
    
    def publish_command(self, data_list):
        msg = Float32MultiArray()
        msg.data = data_list
        self.cmd_publisher.publish(msg)
        
        # For debugging: log the command pattern
        pattern = ",".join([f"{x:.3f}" for x in data_list[:4]])
        self.get_logger().debug(f"Published command: [{pattern}, ...]")

def main(args=None):
    rclpy.init(args=args)
    controller = CableController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
