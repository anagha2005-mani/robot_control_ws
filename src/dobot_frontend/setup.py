from setuptools import setup
import os
from glob import glob

package_name = 'dobot_frontend'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # UI templates
        (os.path.join('share', package_name, 'templates'),
         glob('templates/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Frontend user interfaces for Dobot Magician (ROS 2 Jazzy)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = dobot_frontend.web_server:main',
            'qt_interface = dobot_frontend.qt_interface:main',
        ],
    },
)