import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. get default function package paths
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    # 2. declare a parameter for urdf folder path, convenient for modification
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_xacro_path), description='Loaded model file path'
    )

    # 3. get content through file path, and convert to robot description parameter for robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type=str)
    
    # 4. create robot_state_publisher nodes
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}],
    )

    # 5. include gazebo launch file
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={'world': default_gazebo_world_path, 'verbose': 'true'}.items()
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'fishbot']
    )

    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
        output='screen')
    
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
        output='screen')

    return launch.LaunchDescription([
        action_declare_arg_mode_path, # declare launch argument
        action_robot_state_publisher, # publish robot state (URDF model)
        action_launch_gazebo, # launch gazebo simulator
        action_spawn_entity, # spawn the robot entity in gazebo
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[load_fishbot_effort_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_fishbot_effort_controller,
                on_exit=[load_fishbot_diff_drive_controller],
            )
        ),
    ])