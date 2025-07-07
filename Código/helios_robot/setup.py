from setuptools import find_packages, setup

package_name = 'helios_robot'

setup(
    name=package_name,
    version='1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/helios_robot/launch', ['launch/plot_PCC_launch.py','launch/mocap_optitrack_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaime',
    maintainer_email='jaime.bravo.algaba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "helios_robot_kine_pcc = helios_robot.helios_robot_kine_pcc:main",
            "helios_robot_sensor2pose = helios_robot.helios_robot_sensor2pose:main",
            "helios_calibration = helios_robot.helios_robot_calibration:main",
            "helios_calibration_2 = helios_robot.helios_robot_calibration_2:main",
            "helios_sensors_filter = helios_robot.helios_sensors_filter:main",
            "helios_sensors_filter_2 = helios_robot.helios_sensors_filter_2:main",
            "helios_sensors_filtered_plot = helios_robot.helios_sensors_filtered_plot:main",
            "helios_diff_plot = helios_robot.helios_difference_plot:main",
            "helios_sensors_raw_plot = helios_robot.helios_sensors_raw_plot:main",
            "helios_sensors_normalized = helios_robot.helios_sensors_normalized:main",
            "helios_sensors_merged_plot = helios_robot.helios_sensors_merged_plot:main",
            "helios_sensors_variation = helios_robot.helios_sensors_variation:main",
            "helios_plot_PCC = helios_robot.helios_robot_plot_PCC:main",
            "helios_inversed_kinematic = helios_robot.helios_robot_reversed_kinematic:main",
            "helios_inversed_kinematic_plot = helios_robot.helios_robot_plot_reversed_kinematic:main",
            "helios_robot_sensor2pose_2 = helios_robot.helios_robot_sensor2pose_2:main",
            "helios_mocap = helios_robot.helios_robot_mocap:main",
            "helios_robot_mocap_plot = helios_robot.helios_robot_mocap_plot.main",
            "helios_robot_sensor2angle = helios_robot.helios_robot_sensor2angle:main",
            "helios_pose_plot = helios_robot.helios_pose_plot:main",
            "helios_test = helios_robot.helios_test:main",
            "helios_IK = helios_robot.helios_IK:main",
            "helios_IK_2 = helios_robot.helios_IK_2:main",
            "helios_FK_compare = helios_robot.helios_FK_compare:main",
            "helios_IK_3 = helios_robot.helios_IK_3:main"
       ],
    },
)
 
