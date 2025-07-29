import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import select
import os
import sys

class OppositePublisher(Node):
    def __init__(self):
        super().__init__('opposite_publisher')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_cables_cmd',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float32MultiArray, '/helios_cables_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_msg = None
        self.input_buffer = ""
        self.get_logger().info("Enter 'yes' or 'y' to publish the opposite of the last received message.")

    def listener_callback(self, msg):
        self.last_msg = msg

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(sys.stdin.fileno(), 1024).decode()
            self.input_buffer += data
            while '\n' in self.input_buffer:
                index = self.input_buffer.index('\n')
                line = self.input_buffer[:index].strip()
                self.input_buffer = self.input_buffer[index+1:]
                if line.lower() in ["yes", "y"]:
                    if self.last_msg is not None:
                        new_msg = Float32MultiArray()
                        new_msg.data = [-x for x in self.last_msg.data]
                        self.publisher.publish(new_msg)
                        self.get_logger().info("Published opposite message")
                    else:
                        self.get_logger().warn("No message received yet")

def main(args=None):
    rclpy.init(args=args)
    node = OppositePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
