"""
Component example launch file.

wei <542841336@qq.com>

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
        name='planning_container',
        namespace='planning_component',
        package='rclcpp_components',
        executable='component_container',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='planning_component',
                plugin='planning_component::Publisher',
                name='planning_node',
                namespace='planning_component',
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': True}])
        ]
    )
    ld.add_action(container)

    return ld
