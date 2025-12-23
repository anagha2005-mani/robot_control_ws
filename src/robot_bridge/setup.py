from setuptools import setup, find_packages

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anagha',
    maintainer_email="anaghapathirappilly2005@gmail.com",
    description='Bridge layer between application and controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_bridge = robot_bridge.bridge_node:main',
        ],
    },
)
