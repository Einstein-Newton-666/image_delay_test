from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sensor_data_qos',
            default_value='true'),
        DeclareLaunchArgument(
            name='image_pub_frequency',
            default_value='200'),
        DeclareLaunchArgument(
            name='move_image',
            default_value='true'),
        # true: generate mode 4/5 images in loaned/shared-memory buffers.
        # false: generate a normal cv::Mat first, then copy into those buffers.
        DeclareLaunchArgument(
            name='generate_in_transport_buffer',
            default_value='true'),
        DeclareLaunchArgument(
            name='copy_image',
            default_value='false'),
        DeclareLaunchArgument(
            name='queue_size',
            default_value='1'),
        DeclareLaunchArgument(
            name='mode',
            default_value='4'),
        DeclareLaunchArgument(
            name='use_intra_process_comms',
            default_value='false'),
        # 1: ros2 image_transport, 2: shm_video_transmission,
        # 3: UltraMultiThread, 4: Loaned msg + shm_msg, 5: iceoryx direct.
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
                    extra_arguments=[{
                        'use_intra_process_comms': ParameterValue(
                            LaunchConfiguration('use_intra_process_comms'),
                            value_type=bool),
                    }],
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
                    extra_arguments=[{
                        'use_intra_process_comms': ParameterValue(
                            LaunchConfiguration('use_intra_process_comms'),
                            value_type=bool),
                    }],
                    parameters=[{
                        'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                        'image_pub_frequency': LaunchConfiguration('image_pub_frequency'),
                        'move_image': LaunchConfiguration('move_image'),
                        'generate_in_transport_buffer': LaunchConfiguration(
                            'generate_in_transport_buffer'),
                        'mode': LaunchConfiguration('mode'),
                    }],
                )
            ],
            output='screen',
            emulate_tty=True,
        )
    ])
