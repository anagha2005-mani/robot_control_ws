from setuptools import setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anagha',
    maintainer_email='your_email@example.com',
    description='ROS2 controller for Node.js robot adapter',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = robot_controller.controller_node:main',
        ],
    },
)

