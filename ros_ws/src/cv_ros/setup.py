from setuptools import setup

package_name = 'cv_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/red_color_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Computer vision ROS2 package for PX4 drone',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_color_detector_ros2 = cv_ros.red_color_detector_ros2:main',
            'color_tracking_offboard = cv_ros.color_tracking_offboard:main',
            'offboard_forward_1m = cv_ros.offboard_forward_1m:main',
            'flight_mode_monitor = cv_ros.flight_mode_monitor:main',
            'px4_connection_diagnostics = cv_ros.px4_connection_diagnostics:main',
        ],
    },
)