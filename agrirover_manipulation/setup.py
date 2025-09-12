from setuptools import find_packages, setup

package_name = 'agrirover_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cobot',
    maintainer_email='cobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
    'test': ['pytest'],
	},
    entry_points={
        'console_scripts': [
        	'cmd_serial_node = agrirover_manipulation.cmd_serial_node:main',
        	'task_node = agrirover_manipulation.task_node:main',
        	'goal_sender_node = agrirover_manipulation.goal_sender_node:main',
        	'pick_and_place = agrirover_manipulation.pick_and_place:main',
        ],
    },
)
