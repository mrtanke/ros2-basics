import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # action1: launch other Launch
    multisim_launch_path = [get_package_share_directory('turtlesim'), '/launch/', 'multisim.launch.py']
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path   
        )
    )

    # action2: print data
    action_log_info = launch.actions.LogInfo(msg=str(multisim_launch_path))

    # action3: execute process
    action_topic_list = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'list']
    )

    # action4: organize actions into a group
    action_group = launch.actions.GroupAction([
        # action5: timer action
        launch.actions.TimerAction(period=2.0, actions=[action_include_launch]),
        launch.actions.TimerAction(period=4.0, actions=[action_topic_list]),
    ])

    return launch.LaunchDescription([
        action_log_info,
        action_group,
    ])