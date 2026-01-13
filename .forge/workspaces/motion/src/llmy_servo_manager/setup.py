from setuptools import setup
import os
from glob import glob

package_name = 'llmy_servo_manager'

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
    maintainer_email='dev@leremix.com',
    description='Servo manager for LLMy robot using ST3215 servos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_manager_node = llmy_servo_manager.servo_manager_node:main',
        ],
    },
)