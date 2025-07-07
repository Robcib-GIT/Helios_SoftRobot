from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='helios_robot',
            executable='helios_sensors_filter',
            name='helios_sensors_filter'
        ),
        Node(
            package='helios_robot',
            executable='helios_robot_sensor2pose',
            name='helios_robot_sensor2pose'
        ),
        Node(
            package='helios_robot',
            executable='helios_robot_mocap_plot',
            name='helios_robot_mocap_plot'
        )
    ])

