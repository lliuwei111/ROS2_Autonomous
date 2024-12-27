"""
Component example launch file.

Roberto Masocco <robmasocco@gmail.com>

May 23, 2024
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Builds an example of LaunchDescription for a component container."""
    ld = LaunchDescription()

    #! Create a ComposableNodeContainer, then add ComposableNode instances to it.
    #! Do not forget the use_intra_process_comms argument!
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='camera_components',
        package='rclcpp_components',
        executable='component_container',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='camera_components',
                plugin='camera::Camera',
                name='camera_node',
                namespace='camera_components',
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': True}])
        ]
    )
    ld.add_action(container)

    return ld
