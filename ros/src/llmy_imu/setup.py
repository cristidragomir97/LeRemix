import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'llmy_imu'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='LLMy Team',
    maintainer_email='dev@llmy.dev',
    description='ROS2 IMU driver for LSM6DSOX + MMC5983MA with Madgwick fusion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = llmy_imu.imu_node:main',
        ],
    },
)
