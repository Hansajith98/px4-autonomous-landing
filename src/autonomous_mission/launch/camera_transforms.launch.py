from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '0', '0', '-0.14',
                '0', '1.5707', '0',
                'base_link',
                'camera_link'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical',
            arguments=[
                '0', '0', '0',
                '-1.57079632679', '0', '-1.57079632679',
                'camera_link',
                'camera_optical_frame'
            ],
            output='screen'
        )
    ])