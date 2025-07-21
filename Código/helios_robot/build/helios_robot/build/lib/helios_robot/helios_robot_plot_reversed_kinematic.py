#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from numpy import cos, sin, pi, sqrt, arange, deg2rad, rad2deg, nan_to_num
import numpy as np
import matplotlib.pyplot as plt

def rot_matrix(axis, angle):
    x, y, z = axis
    c = cos(angle)
    s = sin(angle)
    rot_mat = np.array([[c + x**2*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
                        [y*x*(1-c) + z*s, c + y**2*(1-c), y*z*(1-c) - x*s],
                        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z**2*(1-c)]])
    return rot_mat

def parametric_arc(L, theta, N = 10):
    if theta==0 or np.isnan(theta):  # handle invalid/zero angles
        theta = 0.001

    R = L / theta
    theta_i = arange(0, theta, theta/N)
    z = R * sin(theta_i)
    x = -sqrt(R**2 - z**2) + R
    y = np.zeros_like(x)
    return x, y, z

def parametric_pcc(L, theta, phi, N = 10, p0 = [0,0,0]):
    theta_prev = 0
    rot_mat = rot_matrix([0,0,1], phi[0])

    x_combined = []
    y_combined = []
    z_combined = []
    knots = []
    knots.append(p0)

    for i in range(5):
        x, y, z = parametric_arc(L[i], theta[i], N)

        rot_mat = np.dot(rot_mat, rot_matrix([0, 1, 0], theta_prev))
        rot_mat = np.dot(rot_mat, rot_matrix([0, 0, 1], phi[i]))

        pts = np.dot(rot_mat, np.array([x, y, z])) + np.reshape(p0, (3,1))
        x, y, z = pts

        p0 = np.array([x[-1], y[-1], z[-1]])
        theta_prev = theta[i]
        
        x_combined = np.concatenate((x_combined, x))
        y_combined = np.concatenate((y_combined, y))
        z_combined = np.concatenate((z_combined, z))
        
        knots.append(p0)

    return x_combined, y_combined, z_combined, knots

class PccRosNode(Node):
    def __init__(self):
        super().__init__('pcc_ros_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/helios_pose_cmd',
            self.sub_callback,
            10)
        self.theta = np.zeros(5)
        self.phi = np.zeros(5)
        self.L = [6.3, 6.3, 6.3, 6.3, 6.3]
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Setup for matplotlib
        self.fig = plt.figure(figsize=(10,12))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()
        plt.show()

    def sub_callback(self, msg):
        data = np.array(msg.data)
        if data.size >= 10:
            theta_rad = data[:5]
            phi_rad = data[5:10]
            theta_rad = nan_to_num(theta_rad, nan=0.0)
            phi_rad = nan_to_num(phi_rad, nan=0.0)
            self.theta = deg2rad(rad2deg(theta_rad))  # Convert radians to degrees then back to radians
            self.phi = deg2rad(rad2deg(phi_rad))      # Convert radians to degrees then back to radians

    def timer_callback(self):    
        self.ax.cla()
        x, y, z, knots = parametric_pcc(self.L, self.theta, self.phi, N=10, p0=[0,0,0])

        # Reverse the points for plotting
        x_reversed = x[::-1]
        y_reversed = y[::-1]
        z_reversed = z[::-1]
        self.ax.plot(x_reversed, y_reversed, z_reversed, linewidth=2.0, c='k')

        # Reverse knots for consistency
        knots_reversed = knots[::-1]
        end_points_reversed = np.array(knots_reversed)
        self.ax.scatter(end_points_reversed[:,0], end_points_reversed[:,1], end_points_reversed[:,2], c='r', marker='o')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        l = 30
        self.ax.set_xlim([-l, l])
        self.ax.set_ylim([-l, l])
        self.ax.set_zlim(0, l)  # Set Z-axis from 25 (bottom) to 0 (top)

        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = PccRosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
