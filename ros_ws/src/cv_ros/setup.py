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
            'offboard_north_1m = cv_ros.offboard_north_1m:main',
            'flight_mode_monitor = cv_ros.flight_mode_monitor:main',
            'px4_connection_diagnostics = cv_ros.px4_connection_diagnostics:main',
            'gps_position_display = cv_ros.gps_position_display:main',
            'rc_channels_monitor = cv_ros.rc_channels_monitor:main',
            'gps_offboard_publisher = cv_ros.gps_offboard_publisher:main',
            'gps_coordinate_receiver = cv_ros.gps_coordinate_receiver:main',
            'aruco_detector_ros2 = cv_ros.aruco_detector_ros2:main',
            'aruco_tracking_offboard = cv_ros.aruco_tracking_offboard:main',
            'aruco_position_tracking_offboard = cv_ros.aruco_position_tracking_offboard:main',
            'simple_velocity_control = cv_ros.simple_velocity_control:main',
            'velocity_control_test = cv_ros.velocity_control_test:main',
            'aruco_cbtracking = cv_ros.aruco_cbtracking:main',
        ],
    },
)