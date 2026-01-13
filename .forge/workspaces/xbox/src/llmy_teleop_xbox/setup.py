from setuptools import setup, find_packages

package_name = 'llmy_teleop_xbox'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_xbox.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package for Xbox controller teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_xbox = llmy_teleop_xbox.teleop_xbox:main',
            'test_joystick = llmy_teleop_xbox.test_joystick:main',
        ],
    },
)