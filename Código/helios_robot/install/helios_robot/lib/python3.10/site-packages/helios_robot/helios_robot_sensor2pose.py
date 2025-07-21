import os
import tensorflow as tf
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import yaml
from math import pi, sqrt, atan2
import numpy as np

# Define kinematics functions
def rot_matrix(axis, angle):
    """Generate a rotation matrix around a given axis."""
    c, s = np.cos(angle), np.sin(angle)
    if axis == 'z':
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    elif axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.eye(3)

def parametric_arc(L, theta, N=10):
    """Compute points along a parametric arc."""
    t = np.linspace(0, L, N)
    return t * np.cos(theta), t * np.sin(theta)

def parametric_pcc(L_list, theta, phi, N=10, p0=[0, 0, 0]):
    """Compute the position of a parametric chain with given angles."""
    knots = [np.array(p0)]
    R = np.eye(3)
    for i in range(len(L_list)):
        R = R @ rot_matrix('y', phi[i]) @ rot_matrix('z', theta[i])
        p = knots[-1] + R @ np.array([L_list[i], 0, 0])
        knots.append(p)
    return np.array(knots)

def forward_kinematics(angles):
    """Compute the end-effector position from joint angles."""
    theta = angles[:5]  # First 5 angles
    phi = angles[5:]   # Next 5 angles
    L_list = [6.3] * 5  # 5 segments, each 6.3 units
    knots = parametric_pcc(L_list, theta, phi)
    return knots[-1]  # Return the end-effector position

class SensorToPoseNode(Node):
    def __init__(self):
        super().__init__('sensor_to_pose_node')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            'helios_sensors_filtered', 
            self.sensor_callback, 
            10
        )
        # Publisher now uses Point message type
        self.publisher = self.create_publisher(Point, 'helios_pose_meas', 10)

        # Load pretrained AI models
        folder = os.getcwd() + '/src/helios_robot/models/'
        self.models = []
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x40.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x41.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x44.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x45.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x48.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x4A.keras'))

    def normalize(self, data, min_val, max_val):
        return (data - min_val) / (max_val - min_val)
    
    def denormalize(self, data, min_val, max_val):
        return data * (max_val - min_val) + min_val
    
    def sensor_callback(self, msg):
        # Get sensor values
        sensor_values = list(msg.data)
        
        # Initialize theta and phi with 5 elements (matching the loop range)
        theta = [0.0] * 5
        phi = [0.0] * 5
        
        # Predict angles for each position
        for position in range(5):
            if position == 0:
                index = 4
            elif position == 1:
                index = 3
            elif position == 2:
                index = 0
            elif position == 3:
                index = 2
            elif position == 4:
                index = 1

            # Compute differences for input vector
            h02 = sensor_values[index * 4] - sensor_values[index * 4 + 2]
            h13 = sensor_values[index * 4 + 1] - sensor_values[index * 4 + 3]
            input_vector = [h02, h13]

            # Debugging prints (optional, can be removed)
            print("....................................................", flush=True)
            print(f"Index is : {index}", flush=True)
            print(input_vector, flush=True)
            print("....................................................", flush=True)

            # Predict using the model
            prediction = self.models[index](tf.constant([input_vector], dtype=tf.float32))
            euler_y = self.denormalize(float(prediction[0][0]), -60, 60)
            euler_z = self.denormalize(float(prediction[0][1]), -60, 60)

            # Assign predicted angles
            theta[position] = euler_y
            phi[position] = euler_z

        # Compute position from angles
        angles = theta + phi
        position = forward_kinematics(angles)

        # Create and publish Point message
        pose_msg = Point()
        pose_msg.x = position[0]
        pose_msg.y = position[1]
        pose_msg.z = position[2]
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorToPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
