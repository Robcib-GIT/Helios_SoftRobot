import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/carpe-bleue/ROS/helios_ws/src/helios_robot/install/helios_robot'
