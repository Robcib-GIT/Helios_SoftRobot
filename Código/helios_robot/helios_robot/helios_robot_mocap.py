import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class PoseDiffPlotter(Node):
    def __init__(self):
        super().__init__('pose_diff_plotter')
        
        # Explicit QoS profile that matches common motion capture systems
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribers with explicit QoS
        self.base_pose_sub = self.create_subscription(
            PoseStamped,
            'HeliosBase/pose',
            self.base_pose_callback,
            qos_profile=qos_profile
        )
        self.module_pose_sub = self.create_subscription(
            PoseStamped,
            'HeliosModule/pose',
            self.module_pose_callback,
            qos_profile=qos_profile
        )
        
        # Initialize pose storage with timestamps
        self.latest_base_pose = None
        self.latest_module_pose = None
        self.last_base_time = 0
        self.last_module_time = 0
        self.lock = threading.Lock()
        
        # Data storage for plotting
        self.time_data = []
        self.diff_pos = {'x': [], 'y': [], 'z': []}
        self.diff_ori = {'x': [], 'y': [], 'z': [], 'w': []}
        
        # Setup plot
        self.fig, (self.ax_pos, self.ax_ori) = plt.subplots(2, 1, figsize=(10, 8))
        self.setup_plot()
        
        # Timer for sampling at 0.5 seconds
        self.sample_timer = self.create_timer(0.5, self.sample_callback)
        
        # Animation
        self.ani = FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=500,  # Update every 500ms
            cache_frame_data=False
        )
        
    def setup_plot(self):
        # Position plot setup - REDUCED Y-AXIS RANGE TO 0-0.1 METERS
        self.ax_pos.set_title('Absolute Position Differences (0.5s sampling)')
        self.ax_pos.set_ylabel('Difference (m)')
        self.ax_pos.grid(True)
        self.ax_pos.set_ylim(0, 0.1)  # CHANGED FROM 2 TO 0.1
        self.lines_pos = {
            'x': self.ax_pos.plot([], [], 'r-', label='X')[0],
            'y': self.ax_pos.plot([], [], 'g-', label='Y')[0],
            'z': self.ax_pos.plot([], [], 'b-', label='Z')[0]
        }
        self.ax_pos.legend()
        
        # Orientation plot setup
        self.ax_ori.set_title('Absolute Orientation Differences (0.5s sampling)')
        self.ax_ori.set_ylabel('Difference')
        self.ax_ori.set_xlabel('Time (s)')
        self.ax_ori.grid(True)
        self.ax_ori.set_ylim(0, 1.5)
        self.lines_ori = {
            'x': self.ax_ori.plot([], [], 'c-', label='X')[0],
            'y': self.ax_ori.plot([], [], 'm-', label='Y')[0],
            'z': self.ax_ori.plot([], [], 'y-', label='Z')[0],
            'w': self.ax_ori.plot([], [], 'k-', label='W')[0]
        }
        self.ax_ori.legend()
        
        plt.tight_layout()
    
    def base_pose_callback(self, msg):
        with self.lock:
            self.latest_base_pose = msg.pose
            # Convert header time to seconds
            self.last_base_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def module_pose_callback(self, msg):
        with self.lock:
            self.latest_module_pose = msg.pose
            # Convert header time to seconds
            self.last_module_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def sample_callback(self):
        with self.lock:
            if self.latest_base_pose is None or self.latest_module_pose is None:
                return
                
            # Use the latest timestamp from either pose
            current_time = max(self.last_base_time, self.last_module_time)
            
            # Position differences
            pos_diffs = {}
            for axis in ['x', 'y', 'z']:
                base_val = getattr(self.latest_base_pose.position, axis)
                module_val = getattr(self.latest_module_pose.position, axis)
                diff = abs(base_val - module_val)
                pos_diffs[axis] = diff
            
            # Orientation differences
            ori_diffs = {}
            for axis in ['x', 'y', 'z', 'w']:
                base_val = getattr(self.latest_base_pose.orientation, axis)
                module_val = getattr(self.latest_module_pose.orientation, axis)
                diff = abs(base_val - module_val)
                ori_diffs[axis] = diff
            
            # Store data
            self.time_data.append(current_time)
            for axis in pos_diffs:
                self.diff_pos[axis].append(pos_diffs[axis])
            for axis in ori_diffs:
                self.diff_ori[axis].append(ori_diffs[axis])
            
            # Keep a fixed buffer for performance
            buffer_size = 100
            if len(self.time_data) > buffer_size:
                self.time_data.pop(0)
                for axis in self.diff_pos:
                    self.diff_pos[axis].pop(0)
                for axis in self.diff_ori:
                    self.diff_ori[axis].pop(0)
    
    def update_plot(self, frame):
        if not self.time_data:
            return []
        
        # Update position plots
        for axis, line in self.lines_pos.items():
            line.set_data(self.time_data, self.diff_pos[axis])
        
        # Update orientation plots
        for axis, line in self.lines_ori.items():
            line.set_data(self.time_data, self.diff_ori[axis])
        
        # Adjust axes limits dynamically
        min_time = min(self.time_data)
        max_time = max(self.time_data)
        self.ax_pos.set_xlim(min_time, max_time)
        self.ax_ori.set_xlim(min_time, max_time)
        
        # Auto-scale Y-axes (except position which is fixed to 0-0.1)
        self.ax_ori.relim()
        self.ax_ori.autoscale_view(scaley=False)
        
        return list(self.lines_pos.values()) + list(self.lines_ori.values())

def main(args=None):
    rclpy.init(args=args)
    node = PoseDiffPlotter()
    
    # Use multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # Start ROS in separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    plt.show()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
