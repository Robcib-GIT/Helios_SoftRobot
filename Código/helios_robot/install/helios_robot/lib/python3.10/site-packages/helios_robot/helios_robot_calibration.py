import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class CableController(Node):
    def __init__(self):
        super().__init__('cable_controller')
        
        # Declare parameters
        self.declare_parameter('stage', 0)
        self.declare_parameter('direction', 0)
        
        # Get parameters
        self.stage = self.get_parameter('stage').value
        self.direction = self.get_parameter('direction').value
        
        # Validate parameters
        if self.stage not in [0, 1, 2] or self.direction not in [0, 1, 2, 3]:
            self.get_logger().error("Invalid stage or direction")
            self.testing_active = False
        else:
            self.testing_active = True
            self.get_logger().info(f"Starting to pull cable for stage {self.stage}, direction {self.direction}")
        
        # Initialize state variables
        self.initial_values = None
        self.current_values = None
        
        # Create subscriber for /helios_sensors_filtered
        self.sensors_sub = self.create_subscription(
            Float32MultiArray, '/helios_sensors_filtered', self.sensors_callback, 10)
        
        # Create publisher for /helios_cables_cmd
        self.cmd_publisher = self.create_publisher(
            Float32MultiArray, '/helios_cables_cmd', 10)
        
        # Create timer: 1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Cable controller started.")
    
    def sensors_callback(self, msg):
        if self.initial_values is None:
            self.initial_values = list(msg.data)
            self.get_logger().info("Initial sensor values set.")
        self.current_values = list(msg.data)
    
    def timer_callback(self):
        if not self.testing_active or self.initial_values is None:
            cmd = [0.0] * 12
            self.publish_command(cmd)
            return
        
        # Check if any sensor has changed beyond threshold
        for init, curr in zip(self.initial_values, self.current_values):
            if not math.isnan(init) and not math.isnan(curr):
                if abs(curr - init) > 0.03:
                    self.get_logger().info("Sensor change threshold reached. Stopping.")
                    self.testing_active = False
                    cmd = [0.0] * 12 #
                    cable_index = self.stage * 4 + self.direction #
                    cmd[cable_index] = 0.002 #
                    self.publish_command(cmd) #
                    break
        
        if self.testing_active:
            cmd = [0.0] * 12
            cable_index = self.stage * 4 + self.direction
            cmd[cable_index] = -0.001
            self.publish_command(cmd)
        else:
            cmd = [0.0] * 12
            self.publish_command(cmd)
    
    def publish_command(self, data_list):
        msg = Float32MultiArray()
        msg.data = data_list
        self.cmd_publisher.publish(msg)
        # For debugging: log the command pattern
        pattern = ",".join([f"{x:.3f}" for x in data_list])
        self.get_logger().debug(f"Published command: [{pattern}]")

def main(args=None):
    rclpy.init(args=args)
    controller = CableController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
