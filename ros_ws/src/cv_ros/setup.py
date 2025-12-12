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
            'usb_camera_node = cv_ros.usb_camera_node:main',
            'red_detector_downward = cv_ros.red_detector_downward:main',
            'gps_position_controller = cv_ros.gps_position_controller:main',
            'color_tracking_offboard = cv_ros.color_tracking_offboard:main',
        ],
    },
)