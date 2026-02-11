from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    image_size = LaunchConfiguration('image_size')
    pixel_format = LaunchConfiguration('pixel_format')
    output_encoding = LaunchConfiguration('output_encoding')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video50'),  # Camera moved to video50
        DeclareLaunchArgument('image_size', default_value='[640,480]'),
        DeclareLaunchArgument('pixel_format', default_value='MJPG'),  # MJPG gives 30fps vs 15fps YUYV
        DeclareLaunchArgument('output_encoding', default_value='mono8'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[
                {'video_device': video_device},
                {'image_size': image_size},
                {'pixel_format': pixel_format},
                {'output_encoding': output_encoding},
                {'frame_id': frame_id},
            ],
        ),
    ])
