import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import minimize
import math

def rot_matrix(axis, angle):
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    rot_mat = np.array([
        [c + x**2*(1-c),   x*y*(1-c) - z*s, x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s,   c + y**2*(1-c), y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s,   c + z**2*(1-c)]
    ])
    return rot_mat

def parametric_arc(L, theta, N=10):
    if theta == 0 or np.isnan(theta):
        theta = 0.0001
    R = L / theta
    theta_i = np.linspace(0, theta, N)
    z = R * np.sin(theta_i)
    x = -np.sqrt(R**2 - z**2) + R
    y = np.zeros_like(x)
    return x, y, z

def parametric_pcc(L_list, theta, phi, N=10, p0=[0,0,0]):
    theta_prev = 0
    rot_mat = rot_matrix([0,0,1], phi[0])
    x_combined = np.array([])
    y_combined = np.array([])
    z_combined = np.array([])
    knots = [np.array(p0)]
    p0 = np.array(p0)

    for i in range(len(L_list)):
        x, y, z = parametric_arc(L_list[i], theta[i], N)
        rot_mat = np.dot(rot_mat, rot_matrix([0, 1, 0], theta_prev))
        rot_mat = np.dot(rot_mat, rot_matrix([0, 0, 1], phi[i]))
        pts = np.dot(rot_mat, np.vstack([x, y, z])) + p0.reshape((3,1))
        x_trans, y_trans, z_trans = pts
        p0 = np.array([x_trans[-1], y_trans[-1], z_trans[-1]])
        theta_prev = theta[i]
        x_combined = np.concatenate((x_combined, x_trans))
        y_combined = np.concatenate((y_combined, y_trans))
        z_combined = np.concatenate((z_combined, z_trans))
        knots.append(p0)

    return x_combined, y_combined, z_combined, knots

def forward_kinematics(angles):
    theta = angles[:5]
    phi = angles[5:]
    _, _, _, knots = parametric_pcc([6.3]*5, theta, phi, N=10, p0=[0, 0, 0])
    return knots[-1]

def inverse_kinematics(x, y, z):
    target = np.array([x, y, z])
    dist = np.linalg.norm(target)
    max_length = 5 * 6.3
    if dist > max_length:
        return None

    phi0 = math.atan2(y, x)
    frac = min(dist / max_length, 1.0)
    theta0 = math.radians(50) * frac
    guess = [theta0]*5 + [phi0]*5

    def cost(vars):
        pos = forward_kinematics(vars)
        return np.sum((pos - target)**2)

    theta_max = math.radians(50)
    bounds = [(0, theta_max)]*5 + [(-math.pi, math.pi)]*5

    res = minimize(
        cost, guess, bounds=bounds, method='SLSQP',
        options={'ftol': 1e-10, 'eps': 1e-12, 'maxiter': 1000}
    )

    if not res.success:
        print("res not success")
        return None

    final_pos = forward_kinematics(res.x)
    if np.linalg.norm(final_pos - target) > 1e-4:
        print("final_pose - target > 1e-4")
        return None
    print("return res.x.tolist()")
    return res.x.tolist()

class PositionDiffNode(Node):
    def __init__(self):
        super().__init__('position_diff_node')
        self.measured_pose = None
        self.target_pose = None
        self.meas_sub = self.create_subscription(
            Point,
            '/helios_pose_meas',
            self.measured_pose_callback,
            10
        )
        self.target_sub = self.create_subscription(
            Point,
            '/helios_target_position',
            self.target_pose_callback,
            10
        )
        self.angles_publisher = self.create_publisher(Float64MultiArray, 'helios_IK_angles', 10)
        self.timer = self.create_timer(0.1, self.compute_difference)

    def measured_pose_callback(self, msg):
        self.measured_pose = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Received measured pose: x={msg.x}, y={msg.y}, z={msg.z}')

    def target_pose_callback(self, msg):
        self.target_pose = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Received target pose: x={msg.x}, y={msg.y}, z={msg.z}')

    def compute_difference(self):
        if self.measured_pose is None or self.target_pose is None:
            self.get_logger().info('Waiting for both measured and target poses...')
            return
        
        # Compute angles for measured pose
        angles_current = inverse_kinematics(self.measured_pose[0], self.measured_pose[1], self.measured_pose[2])
        if angles_current is None:
            self.get_logger().info('IK solver failed for measured pose.')
            return
        
        # Compute angles for target pose
        angles_target = inverse_kinematics(self.target_pose[0], self.target_pose[1], self.target_pose[2])
        if angles_target is None:
            self.get_logger().info('IK solver failed for target pose.')
            return
        
        # Compute difference: angles_current - angles_target
        difference = []
        for i in range(10):
            if i < 5:
                # For theta angles (first 5), simple subtraction
                diff = angles_current[i] - angles_target[i]
            else:
                # For phi angles (last 5), compute angular difference
                diff = math.atan2(math.sin(angles_current[i] - angles_target[i]), math.cos(angles_current[i] - angles_target[i]))
            difference.append(diff)
        
        self.get_logger().info(f'Computed angle differences: {difference}')
        msg = Float64MultiArray(data=difference)
        self.angles_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionDiffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down position_diff_node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
