import os
import sys
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    bullet_detect_node = ComposableNode(
        package='rm_bullet_detect',
        plugin='qianli_rm_bullet_detect::BulletDetectNode',
        name='bullet_detect_node',
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='bullet_detect_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                bullet_detect_node,
            ],
            output='both',
    )
    print(container)

    return LaunchDescription([container])