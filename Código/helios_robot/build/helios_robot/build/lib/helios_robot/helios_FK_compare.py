import rclpy
from rclpy.node import Node
import numpy as np
import math

class CompareFKNode(Node):
    def __init__(self):
        super().__init__('compare_fk_node')
        # Declare parameters for angles
        self.declare_parameters(
            namespace='',
            parameters=[
                ('theta1', 0.1),
                ('theta2', 0.1),
                ('theta3', 0.1),
                ('theta4', 0.1),
                ('theta5', 0.1),
                ('phi1', 0.2),
                ('phi2', 0.3),
                ('phi3', 0.3),
                ('phi4', 0.5),
                ('phi5', 0.8),
            ]
        )
        self.compare_kinematics()

    def rot_matrix(self, axis, angle):
        """Compute rotation matrix for rotation around axis by angle."""
        axis = np.array(axis) / np.linalg.norm(axis)
        a = math.cos(angle / 2.0)
        b, c, d = -axis * math.sin(angle / 2.0)
        return np.array([
            [a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d + a*c)],
            [2*(b*c + a*d), a*a + c*c - b*b - d*d, 2*(c*d - a*b)],
            [2*(b*d - a*c), 2*(c*d + a*b), a*a + d*d - b*b - c*c]
        ])

    def forward_kinematics(self, angles):
        """Compute end position for a 5-segment continuum robot."""
        num_segs = 5
        L = 6.3
        pos = np.zeros(3)
        R = np.eye(3)

        for i in range(num_segs):
            theta = angles[i]
            phi = angles[num_segs + i]
            if abs(theta) < 1e-7:
                trans_local = np.array([0.0, 0.0, L])
            else:
                r = L / theta
                x = r * (1 - math.cos(theta))
                z = r * math.sin(theta)
                trans_local = np.array([x, 0.0, z])
                axis_local = np.array([-math.sin(phi), math.cos(phi), 0.0])
                R = R @ self.rot_matrix(axis_local, theta)
            pos += R @ trans_local
        return pos

    def parametric_arc(self, L, theta, N):
        """Generate N points along a circular arc in xz-plane."""
        if abs(theta) < 1e-7:
            t = np.linspace(0, L, N)
            x = np.zeros(N)
            y = np.zeros(N)
            z = t
        else:
            r = L / theta
            t = np.linspace(0, theta, N)
            x = r * (1 - np.cos(t))
            y = np.zeros(N)
            z = r * np.sin(t)
        return x, y, z

    def parametric_pcc(self, L, theta, phi, N=10, p0=[0,0,0]):
        """Generate chained arcs for 5 segments, return points and knots."""
        theta_prev = 0
        rot_mat = self.rot_matrix([0,0,1], phi[0])
        x_combined = []
        y_combined = []
        z_combined = []
        knots = []
        knots.append(p0)

        for i in range(5):  # Adjusted to 5 segments
            x, y, z = self.parametric_arc(L[i], theta[i], N)
            rot_mat = np.dot(rot_mat, self.rot_matrix([0, 1, 0], theta_prev))
            rot_mat = np.dot(rot_mat, self.rot_matrix([0, 0, 1], phi[i]))
        
        # Fix: Reshape p0 to be broadcastable with (3,N) array
            points = np.dot(rot_mat, np.array([x, y, z])) + np.array(p0).reshape(3, 1)
        
            x, y, z = points[0], points[1], points[2]
            p0 = np.array([x[-1], y[-1], z[-1]])
            theta_prev = theta[i]
            x_combined = np.concatenate((x_combined, x)) if len(x_combined) > 0 else x
            y_combined = np.concatenate((y_combined, y)) if len(y_combined) > 0 else y
            z_combined = np.concatenate((z_combined, z)) if len(z_combined) > 0 else z
            knots.append(p0)
        return x_combined, y_combined, z_combined, knots

    def compare_kinematics(self):
        """Compare end positions from both functions."""
        # Get parameters
        angles = [
            self.get_parameter(f'theta{i+1}').value for i in range(5)
        ] + [
            self.get_parameter(f'phi{i+1}').value for i in range(5)
        ]
        # Compute forward kinematics
        pos_fk = self.forward_kinematics(angles)
        # Compute parametric PCC
        L = [6.3] * 5
        theta = angles[:5]
        phi = angles[5:]
        _, _, _, knots = self.parametric_pcc(L, theta, phi, N=10, p0=[0,0,0])
        pos_pcc = knots[-1]
        # Compare positions
        match = np.allclose(pos_fk, pos_pcc, atol=1e-5)
        # Log results
        self.get_logger().info(f'Forward Kinematics Position: {pos_fk}')
        self.get_logger().info(f'Parametric PCC Last Point: {pos_pcc}')
        self.get_logger().info(f'Positions Match: {match}')

def main(args=None):
    rclpy.init(args=args)
    node = CompareFKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
