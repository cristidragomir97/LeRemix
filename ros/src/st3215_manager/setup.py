from setuptools import setup
import os
from glob import glob

package_name = 'st3215_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LLMy Team',
    maintainer_email='dev@llmy.dev',
    description='Generic, config-driven ROS2 driver for ST3215 servo motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_manager_node = st3215_manager.servo_manager_node:main',
        ],
    },
)
