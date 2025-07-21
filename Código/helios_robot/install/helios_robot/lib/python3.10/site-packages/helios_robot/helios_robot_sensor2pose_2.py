import os
import tensorflow as tf
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import yaml
from math import pi, sqrt, atan2
import numpy as np 

class SensorToPoseNode(Node):
    def __init__(self):
        super().__init__('sensor_to_pose_node')

        #initially the node subscribes to "helios_sensors"
        self.subscription = self.create_subscription(Float32MultiArray, 'helios_sensors_filtered', self.sensor_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'helios_pose_meas', 10)

        # Load the pretrained AI models
        #folder = os.getcwd() + '/helios_ws/src/helios_robot/models/'
        folder = os.getcwd() + '/src/helios_robot/models/'

        self.models = []
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x40.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x41.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x44.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x45.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x48.keras'))
        self.models.append(tf.keras.models.load_model(folder + 'nn_0x4A.keras'))
        
#        self.models.append(tf.keras.models.load_model(folder + 'nn_0x48.keras'))
#        self.models.append(tf.keras.models.load_model(folder + 'nn_0x45.keras'))
#        self.models.append(tf.keras.models.load_model(folder + 'nn_0x40.keras'))
#        self.models.append(tf.keras.models.load_model(folder + 'nn_0x44.keras'))
#        self.models.append(tf.keras.models.load_model(folder + 'nn_0x41.keras'))
#        self.models.append(tf.keras.models.load_model(folder + 'nn_0x4A.keras'))


    def normalize(self, data, min, max):
        return (data - min) / (max - min)
    
    def denormalize(self, data, min, max):
        return data * (max - min) + min
    
    def sensor_callback(self, msg):
    # Get the sensor values from the message
        sensor_values = msg.data
        sensor_values = list(msg.data)  # Make it a regular list
    
        theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        phi = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Predict the pose. Each model predicts two values: x and y
        for position in range(5): #6 initially, will try 5
            if position == 0 : 
                index = 4
            if position == 1 : 
                index = 3
            if position == 2 : 
                index = 0
            if position == 3 : 
                index = 2
            if position == 4 : 
                index = 1

        # Compute the average of the four sensors
            sensor_avg = (sensor_values[index*4] + sensor_values[index*4+1] + sensor_values[index*4+2] + sensor_values[index*4+3]) / 4.0

            h02 = sensor_values[index*4] - sensor_values[index*4+2]
            h13 = sensor_values[index*4+1] - sensor_values[index*4+3]
            input_vector = [h02, h13]            

            print("....................................................", flush=True)
            print(f"Index is : {index}", flush = True)
            print(input_vector, flush = True)
            print("....................................................", flush=True)

        # Predict the pose
            prediction = self.models[index](tf.constant([input_vector], dtype=tf.float32))
            print("prediction", flush=True)

            euler_y = self.denormalize(float(prediction[0][0]), -60, 60)
            print("euler_y", flush=True)

            euler_z = self.denormalize(float(prediction[0][1]), -60, 60)
            print("euler_z", flush=True)

            theta[position] = np.sqrt(euler_y**2 + euler_z**2)
            phi[position] = np.arctan2(euler_z, euler_y)*180/np.pi
    
    # Convert to radians for computation
        theta_rad = [np.deg2rad(t) for t in theta]
        phi_rad = [np.deg2rad(p) for p in phi]
    
    # Compute the curve with L=[6.3]*6 for 6 segments
        L = [6.3, 6.3, 6.3, 6.3, 6.3, 6.3]
        _, _, _, knots = parametric_pcc(L, theta_rad, phi_rad, N=10, p0=[0,0,0])
    
    # Print the knots' coordinates
        for i, knot in enumerate(knots):
            self.get_logger().info(f"Knot {i}: ({knot[0]:.2f}, {knot[1]:.2f}, {knot[2]:.2f})")
    
    # Create and publish the message with the predicted pose
        pose_msg = Float32MultiArray()
        pose_msg.data = theta + phi
        self.publisher.publish(pose_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = SensorToPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
