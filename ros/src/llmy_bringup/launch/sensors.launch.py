from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable camera (llmy_camera)'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable IMU (llmy_imu)'
    )
    
    use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar',
        default_value='false',
        description='Enable RPLidar C1'
    )

    # Launch configurations
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    use_rplidar = LaunchConfiguration('use_rplidar')

    # Include llmy_camera launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_camera'),
                'launch',
                'camera.launch.py'
            ])
        ),
        launch_arguments=[
            ('enable_compressed', 'false'),
            ('enable_wrist_camera', 'true'),
        ],
        condition=IfCondition(use_camera)
    )

    # Include llmy_imu launch file
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llmy_imu'),
                'launch',
                'imu.launch.py'
            ])
        ),
        condition=IfCondition(use_imu)
    )

    # Include RPLidar launch file
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            ])
        ),
        condition=IfCondition(use_rplidar)
    )

    return LaunchDescription([
        # Launch arguments
        use_camera_arg,
        use_imu_arg,
        use_rplidar_arg,

        # Launch includes
        camera_launch,
        imu_launch,
        rplidar_launch,
    ])