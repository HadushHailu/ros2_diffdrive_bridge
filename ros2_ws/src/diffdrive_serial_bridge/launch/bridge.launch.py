from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    track_width = LaunchConfiguration('track_width')
    max_v = LaunchConfiguration('max_v')
    max_w = LaunchConfiguration('max_w')
    pwm_max = LaunchConfiguration('pwm_max')
    send_hz = LaunchConfiguration('send_hz')
    imu_frame_id = LaunchConfiguration('imu_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('track_width', default_value='0.135'),  # 135mm track width
        DeclareLaunchArgument('max_v', default_value='0.4'),
        DeclareLaunchArgument('max_w', default_value='2.0'),
        DeclareLaunchArgument('pwm_max', default_value='200'),
        DeclareLaunchArgument('send_hz', default_value='30.0'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),

        Node(
            package='diffdrive_serial_bridge',
            executable='bridge_node',
            name='diffdrive_serial_bridge',
            output='screen',
            parameters=[
                {'port': port},
                {'baud': baud},
                {'track_width': track_width},
                {'max_v': max_v},
                {'max_w': max_w},
                {'pwm_max': pwm_max},
                {'send_hz': send_hz},
                {'imu_frame_id': imu_frame_id},
            ],
        ),
    ])
