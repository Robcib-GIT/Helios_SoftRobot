import os
import tensorflow as tf
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import yaml
from math import pi, sqrt, atan2, radians

class SensorToPoseNode(Node):
    def __init__(self):
        super().__init__('sensor_to_pose_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'helios_sensors_filtered',
            self.sensor_callback,
            10)
        self.publisher = self.create_publisher(Float32MultiArray, 'helios_angle_meas', 10)
        folder = os.getcwd() + '/src/helios_robot/models/'
        self.models = []
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x40.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x41.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x44.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x45.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x48.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x4A.keras'))

    def normalize(self, data, min, max):
        return (data - min) / (max - min)
    
    def denormalize(self, data, min, max):
        return data * (max - min) + min
    
    def sensor_callback(self, msg):
        sensor_values = list(msg.data)
        theta = [0.0] * 6
        phi = [0.0] * 6
        
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

            h02 = sensor_values[index*4] - sensor_values[index*4+2]
            h13 = sensor_values[index*4+1] - sensor_values[index*4+3]
            input_vector = [h02, h13]
            
            print("....................................................", flush=True)
            print(f"Index is : {index}", flush=True)
            print(input_vector, flush=True)
            print("....................................................", flush=True)

            prediction = self.models[index](tf.constant([input_vector], dtype=tf.float32))
            print("prediction", flush=True)

            euler_y = self.denormalize(float(prediction[0][0]), -60, 60)
            print("euler_y", flush=True)

            euler_z = self.denormalize(float(prediction[0][1]), -60, 60)
            print("euler_z", flush=True)

            theta[position] = sqrt(euler_y**2 + euler_z**2)
            phi[position] = atan2(euler_z, euler_y) * 180 / pi

        # Convert theta and phi from degrees to radians
        theta_rad = [radians(t) for t in theta]
        phi_rad = [radians(p) for p in phi]

        # Create and publish the message with the predicted pose in radians
        pose_msg = Float32MultiArray()
        pose_msg.data = theta_rad + phi_rad
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorToPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
