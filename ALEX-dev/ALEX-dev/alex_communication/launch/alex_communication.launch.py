from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from alex_utilities.node_checker_service_action import ActivatingNodeAction

from launch.actions import LogInfo, RegisterEventHandler, EmitEvent
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    ld = LaunchDescription()

    alex_communication_param = Path(get_package_share_directory("alex_communication")) / "params" / "alex_communication.yaml"
    device_config_path       = Path(get_package_share_directory("alex_bringup")) / "params" / "device_configuration.yaml"

    # Define the Node
    urp_client_node = Node(
        package="alex_communication",
        executable="urp_client_node",
        name="urp_client_node",
        parameters=[ alex_communication_param ]
    )

    communication_master_node = Node(
        package="alex_communication",
        executable="communication_master",
        name="communication_master",        
        parameters=[ { "external_device_config_path": str(device_config_path) } ]
    )

    mqtt_client_node = Node(
        package="alex_communication",
        executable="mqtt_client_node",
        name="mqtt_client_node",
        parameters=[ alex_communication_param ]
    )

    webrtc_client = Node(
        package="alex_communication",
        executable="webrtc_client",
        name="webrtc_client",
        parameters=[ alex_communication_param ],
    )

    activation_action = ActivatingNodeAction(ld,
        package_name="alex_communication"    
    )

    config_fetcher = LifecycleNode(
        package="alex_communication",
        executable="config_fetcher",
        name="config_fetcher",
        namespace=""
    )

    config_fetcher_node_configure = EmitEvent(
        event=ChangeState(
        lifecycle_node_matcher=matches_action(config_fetcher),
        transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    config_fetcher_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=config_fetcher,
            goal_state='finalized',
            entities=[
                LogInfo(msg=('-----------------------------Config Fetch Completed---------------------')),
                # Add depended actions to LaunchDescription
                communication_master_node   
            ]
        )
    )

    # Add actions to LaunchDescription
    ld.add_action(urp_client_node)
    ld.add_action(mqtt_client_node)
    ld.add_action(webrtc_client)
    ld.add_action(activation_action)
    
    ld.add_action(config_fetcher)
    ld.add_action(config_fetcher_node_configure)
    ld.add_action(config_fetcher_event_handler)

    return ld
