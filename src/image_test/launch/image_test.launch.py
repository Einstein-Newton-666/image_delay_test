from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    mode_is_autoaim_shm = PythonExpression(["'", LaunchConfiguration('mode'), "' == '7'"])

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
        DeclareLaunchArgument(
            name='autoaim_shm_name',
            default_value='/image_test_autoaim_shm_ring'),
        DeclareLaunchArgument(
            name='autoaim_shm_lock_memory',
            default_value='true'),
        # 1: ros2 image_transport, 2: shm_video_transmission,
        # 3: UltraMultiThread, 4: Loaned msg + shm_msg, 5: iceoryx direct,
        # 6: raw rclcpp unique_ptr image, mode 7: auto_aim POSIX shm ring.
        ComposableNodeContainer(
            name='image_test',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=UnlessCondition(mode_is_autoaim_shm),
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
                        'autoaim_shm_name': LaunchConfiguration('autoaim_shm_name'),
                        'autoaim_shm_lock_memory': LaunchConfiguration('autoaim_shm_lock_memory'),
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
                        'autoaim_shm_name': LaunchConfiguration('autoaim_shm_name'),
                        'autoaim_shm_lock_memory': LaunchConfiguration('autoaim_shm_lock_memory'),
                    }],
                )
            ],
            output='screen',
            emulate_tty=True,
        ),
        ComposableNodeContainer(
            name='image_test_autoaim_shm_sub',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(mode_is_autoaim_shm),
            composable_node_descriptions=[
                ComposableNode(
                    package='image_test',
                    plugin='image_test::image_sub',
                    name='image_sub',
                    parameters=[{
                        'copy_image': LaunchConfiguration('copy_image'),
                        'queue_size': LaunchConfiguration('queue_size'),
                        'mode': LaunchConfiguration('mode'),
                        'autoaim_shm_name': LaunchConfiguration('autoaim_shm_name'),
                        'autoaim_shm_lock_memory': LaunchConfiguration('autoaim_shm_lock_memory'),
                    }],
                ),
            ],
            output='screen',
            emulate_tty=True,
        ),
        ComposableNodeContainer(
            name='image_test_autoaim_shm_pub',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(mode_is_autoaim_shm),
            composable_node_descriptions=[
                ComposableNode(
                    package='image_test',
                    plugin='image_test::image_pub',
                    name='image_pub',
                    parameters=[{
                        'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                        'image_pub_frequency': LaunchConfiguration('image_pub_frequency'),
                        'move_image': LaunchConfiguration('move_image'),
                        'generate_in_transport_buffer': LaunchConfiguration(
                            'generate_in_transport_buffer'),
                        'mode': LaunchConfiguration('mode'),
                        'autoaim_shm_name': LaunchConfiguration('autoaim_shm_name'),
                        'autoaim_shm_lock_memory': LaunchConfiguration('autoaim_shm_lock_memory'),
                    }],
                ),
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
