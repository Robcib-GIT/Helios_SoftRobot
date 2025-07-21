#!/usr/bin/env python3
import math
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

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
    print("return.x.tolist()")
    return res.x.tolist()

class HeliosPccNode(Node):
    def __init__(self):
        super().__init__('helios_pcc_node')
        self.subscription = self.create_subscription(
            Point,
            '/helios_target_position',
            self.target_callback,
            10)
        self.theta = np.zeros(5)
        self.phi = np.zeros(5)
        self.L = [6.3]*5
        self.fig = plt.figure(figsize=(8,6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()
        plt.show()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        if math.sqrt(x*x + y*y + z*z) > sum(self.L):
            self.get_logger().info(f"Target ({x:.2f},{y:.2f},{z:.2f}) out of reach. Ignoring.")
            return
        angles = inverse_kinematics(x, y, z)
        if angles is None:
            self.get_logger().info("IK solver failed or target unreachable.")
            return
        self.theta = np.array(angles[:5])
        self.phi = np.array(angles[5:])
        self.get_logger().info("Computed joint angles for target.")

    def timer_callback(self):
        self.ax.cla()
        x_pts, y_pts, z_pts, knots = parametric_pcc(self.L, self.theta, self.phi, N=20, p0=[0,0,0])
        self.ax.plot(x_pts, y_pts, z_pts, linewidth=2.0, c='k')
        end_points = np.array(knots)
        self.ax.scatter(end_points[:,0], end_points[:,1], end_points[:,2], c='r', marker='o')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
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
