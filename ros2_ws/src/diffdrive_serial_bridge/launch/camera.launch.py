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
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),  # YUYV for compatibility
        DeclareLaunchArgument('output_encoding', default_value='rgb8'),  # rgb8 works with YUYV
        DeclareLaunchArgument('frame_id', default_value='camera_link'),

        # Static TF: base_link -> camera_link
        # Camera: 80mm forward, centered, 99mm above ground (89mm plate + 10mm)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.080',
                '--y', '0.0',
                '--z', '0.099',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # V4L2 Camera Node
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
