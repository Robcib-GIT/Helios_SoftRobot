#!/usr/bin/env python3
import math
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

def rot_matrix(axis, angle):
    """Return rotation matrix for rotation about given axis by angle (radians)."""
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    rot_mat = np.array([[c + x**2*(1-c),   x*y*(1-c) - z*s, x*z*(1-c) + y*s],
                        [y*x*(1-c) + z*s,   c + y**2*(1-c), y*z*(1-c) - x*s],
                        [z*x*(1-c) - y*s, z*y*(1-c) + x*s,   c + z**2*(1-c)]])
    return rot_mat

def forward_kinematics(angles):
    """Compute end-effector position given [θ1...θ5, φ1...φ5] in radians."""
    num_segs = 5
    L = 6.3  # segment length
    pos = np.zeros(3)
    R = np.eye(3)  # current orientation matrix

    for i in range(num_segs):
        theta = angles[i]
        phi = angles[num_segs + i]
        if abs(theta) < 1e-7:
            # Straight segment: just translate by [0,0,L] in local frame
            trans_local = np.array([0.0, 0.0, L])
        else:
            r = L / theta
            x = r * (1 - math.cos(theta))
            z = r * math.sin(theta)
            trans_local = np.array([x, 0.0, z])
            # rotation axis in local frame for bending
            axis_local = np.array([-math.sin(phi), math.cos(phi), 0.0])
            R = R @ rot_matrix(axis_local, theta)
        # Translate in world coordinates
        pos += R @ trans_local
    return pos

def inverse_kinematics(x, y, z):
    """Find angles [θ1...θ5, φ1...φ5] to reach target (x,y,z). Returns list or None."""
    target = np.array([x, y, z])
    dist = np.linalg.norm(target)
    max_length = 5 * 6.3
    if dist > max_length:
        return None
    # Initial guess: small bend toward target direction
    phi0 = math.atan2(y, x)
    frac = min(dist / max_length, 1.0)
    theta0 = math.radians(50) * frac
    guess = [theta0]*5 + [phi0]*5

    def cost(vars):
        pos = forward_kinematics(vars)
        err = pos - target
        return np.dot(err, err)

    theta_max = math.radians(50)
    bounds = [(0, theta_max)]*5 + [(-math.pi, math.pi)]*5

    res = minimize(cost, guess, bounds=bounds, method='SLSQP', options={'ftol': 1e-9, 'eps': 1e-11})
    if not res.success:
        return None
    final_pos = forward_kinematics(res.x)
    if np.linalg.norm(final_pos - target) > 1e-5:
        return None
    return res.x.tolist()

def parametric_arc(L, theta, N=10):
    """Compute points of a circular arc for one segment of length L bent by angle theta."""
    if theta == 0 or np.isnan(theta):
        theta = 0.0001  # avoid division by zero
    R = L / theta
    theta_i = np.arange(0, theta, theta/N)
    z = R * np.sin(theta_i)
    x = -np.sqrt(R**2 - z**2) + R
    y = np.zeros_like(x)
    return x, y, z

def parametric_pcc(L_list, theta, phi, N=10, p0=[0,0,0]):
    """Compute combined 3D points of the multi-segment PCC given lists of segment lengths L_list and angles."""
    theta_prev = 0
    rot_mat = rot_matrix([0,0,1], phi[0])
    x_combined = np.array([])
    y_combined = np.array([])
    z_combined = np.array([])
    knots = []
    knots.append(np.array(p0))
    p0 = np.array(p0)

    for i in range(5):
        x, y, z = parametric_arc(L_list[i], theta[i], N)
        # Update rotation matrix for this segment
        rot_mat = np.dot(rot_mat, rot_matrix([0, 1, 0], theta_prev))
        rot_mat = np.dot(rot_mat, rot_matrix([0, 0, 1], phi[i]))
        pts = np.dot(rot_mat, np.vstack([x, y, z])) + p0.reshape((3,1))
        x_trans, y_trans, z_trans = pts
        # End point of this segment
        p0 = np.array([x_trans[-1], y_trans[-1], z_trans[-1]])
        theta_prev = theta[i]
        # Accumulate points
        x_combined = np.concatenate((x_combined, x_trans))
        y_combined = np.concatenate((y_combined, y_trans))
        z_combined = np.concatenate((z_combined, z_trans))
        knots.append(p0)

    return x_combined, y_combined, z_combined, knots

class HeliosPccNode(Node):
    """ROS2 node that computes IK for a target point and plots the resulting PCC path."""
    def __init__(self):
        super().__init__('helios_pcc_node')
        # Subscribe to target position topic
        self.subscription = self.create_subscription(
            Point,
            '/helios_target_position',
            self.target_callback,
            10)
        # Store latest angles for plotting
        self.theta = np.zeros(5)
        self.phi = np.zeros(5)
        self.L = [6.3]*5
        # Set up matplotlib figure for real-time plotting
        self.fig = plt.figure(figsize=(8,6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()
        plt.show()
        # Timer to update the plot at regular intervals
        self.timer = self.create_timer(0.5, self.timer_callback)

    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        # Check reachability (max length = 5*6.3)
        if math.sqrt(x*x + y*y + z*z) > sum(self.L):
            self.get_logger().info(f"Target ({x:.2f},{y:.2f},{z:.2f}) out of reach. Ignoring.")
            return
        # Compute inverse kinematics
        angles = inverse_kinematics(x, y, z)
        if angles is None:
            self.get_logger().info("IK solver failed or target unreachable.")
            return
        # Store the computed angles (first 5 are θ, next 5 are φ)
        self.theta = np.array(angles[:5])
        self.phi = np.array(angles[5:])
        self.get_logger().info("Computed joint angles for target.")

    def timer_callback(self):
        # Clear previous plot
        self.ax.cla()
        # Compute the PCC points for the current angles
        x_pts, y_pts, z_pts, knots = parametric_pcc(self.L, self.theta, self.phi, N=20, p0=[0,0,0])
        # (Optionally reverse the points for plotting consistency)
        x_rev = x_pts[::-1]
        y_rev = y_pts[::-1]
        z_rev = z_pts[::-1]
        # Plot the continuous curve
        self.ax.plot(x_rev, y_rev, z_rev, linewidth=2.0, c='k')
        # Plot the segment end points
        knots_rev = knots[::-1]
        end_points_rev = np.array(knots_rev)
        self.ax.scatter(end_points_rev[:,0], end_points_rev[:,1], end_points_rev[:,2],
                        c='r', marker='o')
        # Label axes
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Set axis limits (Z-axis inverted for view)
        l = sum(self.L)
        self.ax.set_xlim([-l, l])
        self.ax.set_ylim([-l, l])
        self.ax.set_zlim([0, l])
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = HeliosPccNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
