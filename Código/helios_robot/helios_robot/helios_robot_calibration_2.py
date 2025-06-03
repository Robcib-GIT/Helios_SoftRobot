import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_sensors_filtered',
            self.sensor_callback,
            10)
        # Publish motor commands
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/helios_cables_cmd',
            10)
        # Timer to control calibration steps (1 Hz = 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.latest_sensors = None
        # Directions correspond to motor and sensor indices: 0=North, 1=East, 2=South, 3=West
        self.directions = [0, 1, 2, 3]
        self.current_direction_index = 0
        self.state = 'start_calibration'
        self.wait_counter = 0
        self.initial_sum = None
        self.current_command = 0.0

    def sensor_callback(self, msg):
        # Ensure at least 8 sensor values are available
        if len(msg.data) >= 8:
            self.latest_sensors = msg.data[:8]
        else:
            self.latest_sensors = None
            self.get_logger().warn('Sensor data has less than 8 values')

    def timer_callback(self):
        # Continue until all directions are calibrated
        if self.current_direction_index < 4:
            D = self.directions[self.current_direction_index]
            if self.state == 'start_calibration':
                if self.latest_sensors is not None:
                    # Record initial sum for the current direction
                    self.initial_sum = self.latest_sensors[D] + self.latest_sensors[4 + D]
                    self.current_command = 0.001  # Start with 1 mm
                    cmd = [0.0] * 12
                    cmd[D] = self.current_command
                    self.publisher.publish(Float32MultiArray(data=cmd))
                    self.get_logger().info(f'Starting calibration for direction {D} (North=0, East=1, South=2, West=3), initial sum: {self.initial_sum}')
                    self.state = 'calibrating'
                else:
                    self.get_logger().info('Waiting for sensor data')
            elif self.state == 'calibrating':
                if self.latest_sensors is not None:
                    # Compute current sum and difference
                    current_sum = self.latest_sensors[D] + self.latest_sensors[4 + D]
                    difference = abs(current_sum - self.initial_sum)
                    self.get_logger().info(f'Direction {D}, current sum: {current_sum}, difference: {difference}')
                    if difference > 0.005:
                        self.get_logger().info(f'Calibrated direction {D} at command {self.current_command}')
                        self.state = 'waiting'
                        self.wait_counter = 2  # Wait 2 seconds
                    else:
                        # Increase command by 1 mm
                        self.current_command += 0.001
                        cmd = [0.0] * 12
                        cmd[D] = self.current_command
                        self.publisher.publish(Float32MultiArray(data=cmd))
                else:
                    self.get_logger().info('Waiting for sensor data')
            elif self.state == 'waiting':
                self.wait_counter -= 1
                if self.wait_counter <= 0:
                    # Move to next direction
                    self.current_direction_index += 1
                    if self.current_direction_index < 4:
                        self.state = 'start_calibration'
                    else:
                        self.get_logger().info('All directions calibrated')
                        self.timer.cancel()
        else:
            self.get_logger().info('Calibration completed')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
