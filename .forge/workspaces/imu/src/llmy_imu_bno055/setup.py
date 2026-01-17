import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'llmy_imu_bno055'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package for BNO055 9-DOF IMU using smbus2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = llmy_imu_bno055.imu_node:main',
        ],
    },
)
