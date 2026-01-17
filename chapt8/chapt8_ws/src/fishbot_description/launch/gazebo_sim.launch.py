import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get default paths
    robot_name_in_model = "fishbot"
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/custom_room.world'
    # Declare launch parameters
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='Absolute path to the URDF file')
    # Generate robot_description parameter from URDF contents
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
  	
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Include Gazebo launch file via IncludeLaunchDescription
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
      	# Pass arguments to Gazebo
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )
    # Request Gazebo to spawn the robot
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])
    
    # Load and activate fishbot_joint_state_broadcaster controller
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    # Load and activate fishbot_diff_drive_controller controller
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
        output='screen')
    
    delayed_load_joint_state_controller = launch.actions.TimerAction(
        period=5.0,  # Delay 5 seconds to let Gazebo plugins initialize
        actions=[load_joint_state_controller]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        # Event handler executed when spawning finishes   
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[delayed_load_joint_state_controller], # Use the delayed action here
            )
        ),
        # Event handler for load_fishbot_diff_drive_controller
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller],)
        ),
    ])
