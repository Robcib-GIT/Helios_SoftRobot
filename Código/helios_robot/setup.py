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
            "helios_sensors_variation = helios_robot.helios_sensors_variation:main"
       ],
    },
)
 
