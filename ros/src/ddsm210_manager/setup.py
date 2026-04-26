from setuptools import setup
import os
from glob import glob

package_name = 'ddsm210_manager'

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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='LLMy Team',
    maintainer_email='dev@llmy.dev',
    description='ROS2 driver for Waveshare DDSM210 direct drive servo motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ddsm210_node = ddsm210_manager.ddsm210_node:main',
            'terminal = ddsm210_manager.terminal:main',
        ],
    },
)
