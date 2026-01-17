from setuptools import setup
import os
from glob import glob

package_name = 'llmy_mcp'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'fastmcp'],
    zip_safe=True,
    maintainer='LLMy Team',
    maintainer_email='dev@llmy.dev',
    description='MCP server for LLMy robot - exposes ROS2 interfaces to LLMs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcp_server = llmy_mcp.server:main',
        ],
    },
)
