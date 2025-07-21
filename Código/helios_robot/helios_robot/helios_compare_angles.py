import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class HeliosPoseCmdNode(Node):
    def __init__(self):
        super().__init__('helios_pose_cmd_node')
        self.sub_meas = self.create_subscription(
            Float32MultiArray,
            '/helios_angle_meas',
            self.meas_callback,
            10)
        self.sub_IK = self.create_subscription(
            Float32MultiArray,
            '/helios_IK_angles',
            self.IK_callback,
            10)
        self.pub = self.create_publisher(Float32MultiArray, '/helios_pose_cmd', 10)
        self.meas_received = False
        self.IK_received = False
        self.theta_1_meas = 0.0
        self.theta_2_meas = 0.0
        self.theta_3_meas = 0.0
        self.phi_1_meas = 0.0
        self.phi_2_meas = 0.0
        self.phi_3_meas = 0.0
        self.theta_1_IK = 0.0
        self.theta_2_IK = 0.0
        self.theta_3_IK = 0.0
        self.phi_1_IK = 0.0
        self.phi_2_IK = 0.0
        self.phi_3_IK = 0.0

    def meas_callback(self, msg):
        if len(msg.data) < 11:
            self.get_logger().error('meas message has insufficient data')
            return
        thetas = msg.data[0:5]
        phis = msg.data[6:11]
        self.theta_1_meas = thetas[0] + thetas[1]
        self.theta_2_meas = thetas[2] + thetas[3]
        self.theta_3_meas = thetas[4]
        self.phi_1_meas = (phis[0] + phis[1]) / 2.0
        self.phi_2_meas = (phis[2] + phis[3]) / 2.0
        self.phi_3_meas = phis[4]
        self.meas_received = True
        self.publish_fused()

    def IK_callback(self, msg):
        if len(msg.data) < 10:
            self.get_logger().error('IK message has insufficient data')
            return
        thetas = msg.data[0:5]
        phis = msg.data[5:10]
        self.theta_1_IK = thetas[0] + thetas[1]
        self.theta_2_IK = thetas[2] + thetas[3]
        self.theta_3_IK = thetas[4]
        self.phi_1_IK = (phis[0] + phis[1]) / 2.0
        self.phi_2_IK = (phis[2] + phis[3]) / 2.0
        self.phi_3_IK = phis[4]
        self.IK_received = True
        self.publish_fused()

    def publish_fused(self):
        if not self.meas_received or not self.IK_received:
            return
        theta_1 = (-self.theta_1_meas + self.theta_1_IK)
        theta_2 = (-self.theta_2_meas + self.theta_2_IK)
        theta_3 = (-self.theta_3_meas + self.theta_3_IK)
        phi_1 = (self.phi_1_meas + self.phi_1_IK) / 2.0
        phi_2 = (self.phi_2_meas + self.phi_2_IK) / 2.0
        phi_3 = (self.phi_3_meas + self.phi_3_IK) / 2.0
        msg = Float32MultiArray()
        msg.data = [theta_1, theta_2, theta_3, phi_1, phi_2, phi_3]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeliosPoseCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
