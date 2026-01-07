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

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files (if you have any)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # Templates for the web frontend
        (os.path.join('share', package_name, 'templates'),
         glob('templates/*.html')),
    ],
    install_requires=['setuptools', 'flask', 'flask_cors', 'flask_socketio'],
    zip_safe=True,
    maintainer='Anagha',           # optional, can be left as 'Your Name'
    maintainer_email='anagha@example.com',  # optional, can be left as placeholder
    description='Frontend web interface (Flask + ROS2) for Dobot Magician',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = dobot_frontend.web_server:main',
            'qt_interface = dobot_frontend.qt_interface:main',  # keep if you have QT UI
        ],
    },
)