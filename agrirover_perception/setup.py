from setuptools import setup

package_name = 'agrirover_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 camera node for tomato detection and pose estimation',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'camera_node = agrirover_perception.camera_node:main',
        ],
    },
)
