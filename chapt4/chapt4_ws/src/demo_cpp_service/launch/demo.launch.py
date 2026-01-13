import launch
import launch_ros

def generate_launch_description():
    # 1. declare a launch parameter
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument(
        'launch_arg_bg',
        default_value='150',
        description='Background green color component'
    )

    # 2. pass the launch parameter to a certain node
    """Generate launch desctription."""
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[{'background_g': launch.substitutions.LaunchConfiguration('launch_arg_bg', default='150')}],
        name='screen'
    )
    action_node_partol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='partol_client',
        name='log'
    )
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        name='both'
    )

    return launch.LaunchDescription([
        action_declare_arg_background_g,
        action_node_turtlesim_node,
        action_node_partol_client,
        action_node_turtle_control,
    ])