import os

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sensor_data_qos',
                    default_value='true'),
        DeclareLaunchArgument(name='image_pub_frequency',
                    default_value='200'),
        DeclareLaunchArgument(name='move_image',
                    default_value='true'),
        DeclareLaunchArgument(name='copy_image',
                    default_value='false'),
        DeclareLaunchArgument(name='queue_size',
                    default_value='1'),
        DeclareLaunchArgument(name='mode',
                    default_value='3'),
                    # 1:ros 2:shm_video_transmission 3:UltraMultiThread
        ComposableNodeContainer(
            name='image_test',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_test',
                    plugin='image_test::image_sub',
                    name='image_sub',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    parameters=[{
                        'copy_image': LaunchConfiguration('copy_image'),
                        'queue_size': LaunchConfiguration('queue_size'),
                        'mode': LaunchConfiguration('mode'),
                    }],
                ),
                ComposableNode(
                    package='image_test',
                    plugin='image_test::image_pub',
                    name='image_pub',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    parameters=[{
                        'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                        'image_pub_frequency': LaunchConfiguration('image_pub_frequency'),
                        'move_image': LaunchConfiguration('move_image'),
                        'mode': LaunchConfiguration('mode'),
                    }],
                )
            ],
            output='screen',
            emulate_tty=True,
        )
    ])


