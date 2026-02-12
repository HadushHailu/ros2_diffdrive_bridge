from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Static TF: base_link -> imu_link
        # IMU: centered front-to-back, 62mm to the right, 89mm above ground (plate height)
        # IMU is flipped (180° around X): quaternion (1, 0, 0, 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=[
                '--x', '0.0',
                '--y', '-0.062',
                '--z', '0.089',
                '--qx', '1.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),
    ])
