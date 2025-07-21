import math
import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

def forward_kinematics(angles):
    """Compute end-effector position given [θ1...θ5, φ1...φ5] (radians)."""
    num_segs = 5
    L = 6.3  # segment length
    pos = np.zeros(3)
    R = np.eye(3)  # current orientation matrix
    # Build rotation matrix for an axis and angle (Rodrigues' formula)
    def rot_matrix(axis, ang):
        axis = axis / np.linalg.norm(axis)
        K = np.array([[    0,     -axis[2],  axis[1]],
                      [ axis[2],     0,     -axis[0]],
                      [-axis[1],  axis[0],     0   ]])
        return np.eye(3) + np.sin(ang)*K + (1-np.cos(ang))*(K @ K)

    # Iterate over each segment
    for i in range(num_segs):
        theta = angles[i]
        phi = angles[num_segs + i]
        if abs(theta) < 1e-6:
            # Straight segment: just translate by [0,0,L] in local frame
            trans_local = np.array([0.0, 0.0, L])
        else:
            r = L / theta
            x = r * (1 - math.cos(theta))
            z = r * math.sin(theta)
            trans_local = np.array([x, 0.0, z])
            # rotation axis in local frame for bending
            axis_local = np.array([-math.sin(phi), math.cos(phi), 0.0])
            # Update orientation (in local coordinates)
            R = R @ rot_matrix(axis_local, theta)
        # Translate in world coordinates
        pos += R @ trans_local
    return pos

def inverse_kinematics(x, y, z):
    """Find angles [θ1...φ5] to reach target (x,y,z). Returns list or None."""
    # Initial guess: small bends towards the target's horizontal angle
    target = np.array([x, y, z])
    dist = np.linalg.norm(target)
    # Quick reach check: total length = 5*6.3
    if dist > 31.5:
        return None
    # Initial guesses
    phi0 = math.atan2(y, x)
    frac = min(dist / 31.5, 1.0)
    theta0 = math.radians(50) * frac  # distribute needed bend
    guess = [theta0]*5 + [phi0]*5

    # Define residual function (error to target)
    def residuals(vars):
        pos = forward_kinematics(vars)
        return pos - target

    # Bounds: 0<=θ<=50°, -π<=φ<=π
    theta_max = math.radians(50)
    lb = [0]*5 + [-math.pi]*5
    ub = [theta_max]*5 + [math.pi]*5
    bounds = (lb, ub)

    res = least_squares(residuals, guess, bounds=bounds, method='trf')
    if not res.success or np.linalg.norm(res.fun) > 1e-2:
        return None
    return res.x.tolist()

class HeliosIKNode(Node):
    """ROS2 node that subscribes to target points and publishes joint angles."""
    def __init__(self):
        super().__init__('helios_ik_node')
        # Subscriber: target position
        self.subscription = self.create_subscription(
            Point,
            '/helios_target_position',
            self.target_callback,
            10)
        # Publisher: angles command
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/helios_pose_cmd',
            10)

    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        # Check reachability
        if math.sqrt(x*x + y*y + z*z) > 31.5:
            self.get_logger().info(f"Target ({x:.2f},{y:.2f},{z:.2f}) out of reach. Ignoring.")
            return
        # Solve IK
        angles = inverse_kinematics(x, y, z)
        if angles is None:
            self.get_logger().info("IK solver failed or target unreachable.")
            return
        # Prepare message with 10 values: [θ1, φ1, ..., θ5, φ5]
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
