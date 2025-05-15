from launch import LaunchDescription, logging
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
from alex_utilities.common_utilities import get_device_namespace
from alex_utilities.node_checker_service_action import ActivatingNodeAction
from alex_utilities.device_configuration_handler import ExternalDeviceConfiguration, ConfigValidationError

def load_nodes_from_config(ld: LaunchDescription, device_config_path: Path):
    """
    Load nodes from the device configuration file and add them to the launch description.
    """

    launch_logger = logging.get_logger('alex_externals.launch')

    try:
        # Load the device configuration
        configuration = ExternalDeviceConfiguration.parse_from_yaml_path(device_config_path)
    except ConfigValidationError as e:
        launch_logger.error(f"Configuration validation error: {e}")
        return

    # Iterate through the devices and their external nodes
    for device_name, device_config in configuration.devices.items():
        for node in device_config.external_nodes:
            launch_logger.info(f"Device: {device_name} | Node: {node.node_name}")
            # Construct the namespace
            namespace = f"{device_name}/" + node.construct_namespace_prefix()
            namespace = get_device_namespace(namespace)

            # Add the node to the launch description
            ld.add_action(
                Node(
                    package="alex_externals",
                    executable=node.node_name,
                    name=node.node_name,
                    namespace=namespace,
                    parameters=[node.parameters, {"device_id": device_name}],
                )
            )


def generate_launch_description():

    ld = LaunchDescription()

    # Path to the device configuration file
    device_config_path = Path(get_package_share_directory("alex_bringup")) / "params" / "device_configuration.yaml"

    device_manager = Node(
        package="alex_externals",
        executable="device_manager",
        name="device_manager",
        parameters=[ { "external_device_config_path": str(device_config_path) } ],
    )

    activation_action = ActivatingNodeAction(ld,
        package_name="alex_externals",
    )
    
    load_nodes_from_config(ld, device_config_path)

    ld.add_action(activation_action)
    ld.add_action(device_manager)

    return ld
