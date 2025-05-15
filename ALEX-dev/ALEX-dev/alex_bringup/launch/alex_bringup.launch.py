from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch.event_handlers import OnProcessExit
from launch.actions import LogInfo, RegisterEventHandler

from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()

    # Get the path to packages
    alex_communication_launch = Path(get_package_share_directory('alex_communication')) / "launch" / 'alex_communication.launch.py'
    alex_controller_launch    = Path(get_package_share_directory('alex_controller'))    / "launch" / 'alex_controller.launch.py'
    alex_externals_launch     = Path(get_package_share_directory('alex_externals'))     / "launch" / 'alex_externals.launch.py'

    # Include the launch file
    launch_alex_communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(alex_communication_launch)]),
    )
    launch_alex_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(alex_controller_launch)]),
    )
    launch_alex_externals = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(alex_externals_launch)]),
    )
    
    checker_node = Node(
        package="alex_bringup",
        executable="node_checker",
        name="node_checker",
    )

    startup_diagnostics_node = Node(
        package="alex_bringup",
        executable="startup_diagnostics",
        name="startup_diagnostics",
    )

    waiting_node = Node(
        package="alex_bringup",
        executable="lifecycle_node_waiter",
        name="lifecycle_node_waiter"
    )

    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=waiting_node,
            on_exit=[
                LogInfo(msg=('-----------------------------CLOSED---------------------')),
                launch_alex_controller,
                launch_alex_externals,
            ]
        )
    )

    # Adding Nodes
    ld.add_action(checker_node)
    ld.add_action(startup_diagnostics_node)
    # Add launch files to LaunchDescription
    ld.add_action(launch_alex_communication)
    
    ld.add_action(waiting_node)
    ld.add_action(event_handler)

    return ld
