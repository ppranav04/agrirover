from setuptools import setup

package_name = 'agrirover_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Manipulation package for agrirover - serial control to Arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_motor_commander = agrirover_manipulation.serial_motor_commander:main',
            'ik_solver = agrirover_manipulation.ik_solver:main',
            'visualization_node = agrirover_manipulation.visualization_node:main',
            'robot_state_publisher_node = agrirover_manipulation.robot_state_publisher_node:main',
            'interactive_marker_server = agrirover_manipulation.interactive_marker_server:main',
            'goal_pose_input = agrirover_manipulation.goal_pose_input:main',
        ],
    },
)
