import math
import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

def forward_kinematics(angles):
    """Compute end-effector position given [θ1...φ5] (radians)."""
    num_segs = 5
    L = 6.3  # segment length
    pos = np.zeros(3)
    R = np.eye(3)  # current orientation matrix

    def rot_matrix(axis, ang):
        axis = axis / np.linalg.norm(axis)
        K = np.array([[    0,     -axis[2],  axis[1]],
                      [ axis[2],     0,     -axis[0]],
                      [-axis[1],  axis[0],     0   ]])
        return np.eye(3) + np.sin(ang)*K + (1-np.cos(ang))*(K @ K)

    for i in range(num_segs):
        theta = angles[i]
        phi = angles[num_segs + i]
        if abs(theta) < 1e-6:
            trans_local = np.array([0.0, 0.0, L])
        else:
            r = L / theta
            x = r * (1 - math.cos(theta))
            z = r * math.sin(theta)
            trans_local = np.array([x, 0.0, z])
            axis_local = np.array([-math.sin(phi), math.cos(phi), 0.0])
            R = R @ rot_matrix(axis_local, theta)
        pos += R @ trans_local
    return pos

def inverse_kinematics(x, y, z):
    """Find angles [θ1...φ5] to reach target (x,y,z). Returns list or None."""
    target = np.array([x, y, z])
    dist = np.linalg.norm(target)
    if dist > 31.5:
        return None

    phi0 = math.atan2(y, x)
    frac = min(dist / 31.5, 1.0)
    theta0 = math.radians(50) * frac
    guess = [theta0]*5 + [phi0]*5

    def residuals(vars):
        return forward_kinematics(vars) - target

    theta_max = math.radians(50)
    lower_bounds = [0]*5 + [-math.pi]*5
    upper_bounds = [theta_max]*5 + [math.pi]*5

    res = least_squares(
        fun=residuals,
        x0=guess,
        bounds=(lower_bounds, upper_bounds),
        method='trf',
        xtol=1e-8,
        ftol=1e-8,
        gtol=1e-8
    )

    if not res.success:
        return None
    if np.linalg.norm(res.fun) > 1e-2:
        return None
    return res.x.tolist()

class HeliosIKNode(Node):
    """ROS2 node that subscribes to target points and publishes joint angles."""
    def __init__(self):
        super().__init__('helios_ik_node')
        self.subscription = self.create_subscription(
            Point,
            '/helios_target_position',
            self.target_callback,
            10)
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/helios_pose_cmd',
            10)

    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        if math.sqrt(x*x + y*y + z*z) > 31.5:
            self.get_logger().info(f"Target ({x:.2f},{y:.2f},{z:.2f}) out of reach. Ignoring.")
            return
        angles = inverse_kinematics(x, y, z)
        if angles is None:
            self.get_logger().info("IK solver failed or target unreachable.")
            return
        pose_msg = Float32MultiArray()
        pose_msg.data = [float(val) for val in angles]
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f"Published angles: {pose_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = HeliosIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
