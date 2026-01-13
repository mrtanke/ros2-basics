import launch
import launch_ros

def generate_launch_description():
    """Generate launch desctription."""
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='screen'
    )
    action_node_partol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='partol_client',
        name='both'
    )
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        name='both'
    )

    return launch.LaunchDescription([
        action_node_turtlesim_node,
        action_node_partol_client,
        action_node_turtle_control,
    ])