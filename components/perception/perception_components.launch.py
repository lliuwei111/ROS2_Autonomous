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
        name='perception_container',
        namespace='perception_component',
        package='rclcpp_components',
        executable='component_container',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='perception_component',
                plugin='perception_component::Publisher',
                name='perception_node',
                namespace='perception_component',
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': True}])
        ]
    )
    ld.add_action(container)

    return ld
