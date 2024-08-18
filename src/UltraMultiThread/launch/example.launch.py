import os

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        ComposableNodeContainer(
            name='umt_example',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='UltraMultiThread',
                    plugin='umt_example::example_sub',
                    name='example_sub',
                ),
                ComposableNode(
                    package='UltraMultiThread',
                    plugin='umt_example::example_pub',
                    name='example_pub',
                )
            ],
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='UltraMultiThread',
        #     executable='example_sub',
        #     name='example_sub',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        # Node(
        #     package='UltraMultiThread',
        #     executable='example_pub',
        #     name='example_pub',
        #     output='screen',
        #     emulate_tty=True,
        # )
    ])


