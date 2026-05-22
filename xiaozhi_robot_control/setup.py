from glob import glob
from os.path import join

from setuptools import setup


package_name = 'xiaozhi_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (join('share', package_name, 'config'), glob('config/*.xml')),
        (join('share', package_name, 'scripts'), glob('scripts/*.sh') + glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diablo',
    maintainer_email='diablo@example.com',
    description='Xiaozhi MCP bridge for controlling the Diablo ROS2 robot.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_mcp_server = xiaozhi_robot_control.robot_mcp_server:main',
        ],
    },
)
