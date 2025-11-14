**Helios SoftRobot**
=====================================
![Demo](Media/Vídeos/demo_movimiento.gif)

**Installation and Configuration**
-------------------------------

### Step 1: Clone the Helios repository

Clone the Helios repository using Git:

```bash
git clone https://github.com/JaimeBravoAlgaba/Helios_SoftRobot.git
```

### Step 2: Create a ROS2 workspace

Create a directory for the ROS2 workspace and make sure it's empty:

```bash
mkdir -p helios_ws/src
```

### Step 3: Install the helios_robot package in the workspace

Copy the helios_robot package to the ROS2 workspace:

```bash
cp -r Helios_SoftRobot/Código/helios_robot helios_ws/src
cd helios_ws/src
```

### Step 4: Install the micro_ros_setup package

Clone the micro_ros_setup repository and make sure it's on the branch corresponding to your ROS2 version:

```bash
git clone https://github.com/micro-ROS/micro_ros_setup.git -b $ROS_DISTRO
cd ..
```

### Step 5: Build the workspace

Update dependencies and build the ROS2 workspace:

```bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/local_setup.bash
```

### Step 6: Create a MicroROS agent

Create a MicroROS agent and configure the environment:

```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```


### Step 7: Connect to the ESP32 and run the agent

Change your IP address to 192.168.2.76:

![image](https://github.com/user-attachments/assets/b8df0da8-761b-4295-9cd1-7558f4a52867)

Turn on the robot if you haven't already.

**Terminal Control**
-----------
- You can use the helios_robot.sh file to automatically launch the necessary nodes:
```bash
# Launches the MicroROS agent, kinematics and pose prediction
# In the folder where the file is located, run:
./helios_robot.sh
```
and immediately after, click the RESET button on the ESP32.

- Launch the visualization node:
```bash
ros2 run helios_robot helios_robot_plot
```

- Control the cable lengths, in meters:
```bash
# [L00, L01, L02, L03, L10, L11, ... L22, L23]
ros2 topic pub --once /helios_cables_cmd std_msgs/msg/Float32MultiArray "data: [0,0,0,0,  0,0,0,0,  0,0,0,0]"
```

- Send command in theta and phi coordinates, in radians:
```bash
# [th0, th1, th2, phi0, phi1, phi2]
ros2 topic pub --once /helios_sections_cmd std_msgs/msg/Float32MultiArray "data: [0,0,0,  0,0,0]"
```

- Control the tool (value from 0 to 255):
```bash
ros2 topic pub --once /helios_tool_cmd std_msgs/msg/Int8 "data: 127"
```


### Alternative: Connect to ESP32 via USB

You can use the Helios with a USB cable:
Connect the ESP32 to the USB port and upload the esp32 project after uncommenting the lines that allow serial connection in "main.cpp" and changing board_microros_transport to "serial" in the platformio.ini file

Run the MicroROS agent with:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```


### Common Error Resolution

If an error appears when connecting the ESP32, make sure to grant your user access to the ttyUSB0 USB port:

1. Check the user group that has access to the port:
```bash
stat /dev/ttyUSB0
```

It should show something like: "Gid: (   20/ dialout)"

2. Add your user to the "dialout" group or whichever group appeared:
```bash
sudo usermod -a -G dialout $USER
```
3. Grant read and write permissions to the port:
```bash
sudo chmod a+rw /dev/ttyUSB0
```

After resetting the ESP32, its topics should be visible from another terminal.
