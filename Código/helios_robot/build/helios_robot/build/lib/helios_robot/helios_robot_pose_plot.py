import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'helios_pose_meas',
            self.listener_callback,
            10)
        
        # Initialize plots for each pair
        self.fig1, self.ax1 = plt.subplots()
        self.ax1.set_xlabel('theta1')
        self.ax1.set_ylabel('phi1')
        self.line1, = self.ax1.plot([], [], 'ro', label='theta1 vs phi1')
        self.ax1.legend()
        
        self.fig2, self.ax2 = plt.subplots()
        self.ax2.set_xlabel('theta2')
        self.ax2.set_ylabel('phi2')
        self.line2, = self.ax2.plot([], [], 'bo', label='theta2 vs phi2')
        self.ax2.legend()
        
        self.fig3, self.ax3 = plt.subplots()
        self.ax3.set_xlabel('theta3')
        self.ax3.set_ylabel('phi3')
        self.line3, = self.ax3.plot([], [], 'go', label='theta3 vs phi3')
        self.ax3.legend()
        
        self.fig4, self.ax4 = plt.subplots()
        self.ax4.set_xlabel('theta4')
        self.ax4.set_ylabel('phi3')
        self.line4, = self.ax4.plot([], [], 'co', label='theta4 vs phi3')
        self.ax4.legend()
        
        self.fig5, self.ax5 = plt.subplots()
        self.ax5.set_xlabel('theta5')
        self.ax5.set_ylabel('phi5')
        self.line5, = self.ax5.plot([], [], 'mo', label='theta5 vs phi5')
        self.ax5.legend()
        
        # Data storage for each pair
        self.data1 = ([], [])  # (theta1, phi1)
        self.data2 = ([], [])  # (theta2, phi2)
        self.data3 = ([], [])  # (theta3, phi3)
        self.data4 = ([], [])  # (theta4, phi3)
        self.data5 = ([], [])  # (theta5, phi5)
        
        plt.ion()  # Enable interactive mode for dynamic updates

    def listener_callback(self, msg):
        data = msg.data
        if len(data) < 10:
            self.get_logger().warn('Received message with insufficient data')
            return
        
        # Extract and plot each pair if not NaN
        if not np.isnan(data[0]) and not np.isnan(data[6]):
            self.data1[0].append(data[0])
            self.data1[1].append(data[6])
            self.line1.set_data(self.data1[0], self.data1[1])
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.fig1.canvas.draw()
            self.fig1.canvas.flush_events()
        
        if not np.isnan(data[1]) and not np.isnan(data[7]):
            self.data2[0].append(data[1])
            self.data2[1].append(data[7])
            self.line2.set_data(self.data2[0], self.data2[1])
            self.ax2.relim()
            self.ax2.autoscale_view()
            self.fig2.canvas.draw()
            self.fig2.canvas.flush_events()
        
        if not np.isnan(data[2]) and not np.isnan(data[8]):
            self.data3[0].append(data[2])
            self.data3[1].append(data[8])
            self.line3.set_data(self.data3[0], self.data3[1])
            self.ax3.relim()
            self.ax3.autoscale_view()
            self.fig3.canvas.draw()
            self.fig3.canvas.flush_events()
        
        if not np.isnan(data[3]) and not np.isnan(data[8]):
            self.data4[0].append(data[3])
            self.data4[1].append(data[8])
            self.line4.set_data(self.data4[0], self.data4[1])
            self.ax4.relim()
            self.ax4.autoscale_view()
            self.fig4.canvas.draw()
            self.fig4.canvas.flush_events()
        
        if not np.isnan(data[4]) and not np.isnan(data[9]):
            self.data5[0].append(data[4])
            self.data5[1].append(data[9])
            self.line5.set_data(self.data5[0], self.data5[1])
            self.ax5.relim()
            self.ax5.autoscale_view()
            self.fig5.canvas.draw()
            self.fig5.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    pose_plotter = PosePlotter()
    rclpy.spin(pose_plotter)
    pose_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
