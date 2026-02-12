from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('diffdrive_serial_bridge')

    # Bridge parameters
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    track_width = LaunchConfiguration('track_width')

    # Camera parameters
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('track_width', default_value='0.135'),
        DeclareLaunchArgument('video_device', default_value='/dev/video50'),

        # 1. IMU static TF publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'imu.launch.py'])
            ),
        ),

        # 2. Camera (includes camera TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'camera.launch.py'])
            ),
            launch_arguments={'video_device': video_device}.items(),
        ),

        # 3. Bridge node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'bridge.launch.py'])
            ),
            launch_arguments={
                'port': port,
                'baud': baud,
                'track_width': track_width,
            }.items(),
        ),
    ])
