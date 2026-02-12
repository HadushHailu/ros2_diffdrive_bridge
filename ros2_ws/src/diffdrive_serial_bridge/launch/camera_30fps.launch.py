from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    pixel_format = LaunchConfiguration('pixel_format')
    camera_name = LaunchConfiguration('camera_name')
    camera_info_url = LaunchConfiguration('camera_info_url')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video17'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),  # MJPG decoded to RGB
        DeclareLaunchArgument('camera_name', default_value='default_cam'),
        DeclareLaunchArgument('camera_info_url', 
            default_value='file:///home/hadush/.ros/camera_info/default_cam.yaml'),

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

        # USB Camera Node (supports MJPG decoding)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'image_width': image_width,
                'image_height': image_height,
                'framerate': framerate,
                'pixel_format': pixel_format,
                'camera_name': camera_name,
                'camera_info_url': camera_info_url,
                'frame_id': 'camera_link',
                'io_method': 'mmap',
            }],
            remappings=[
                ('image_raw', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
        ),
    ])
