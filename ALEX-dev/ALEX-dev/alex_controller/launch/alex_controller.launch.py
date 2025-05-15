from launch import LaunchDescription, logging
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
from alex_utilities.node_checker_service_action import ActivatingNodeAction
from alex_utilities.device_configuration_handler import ExternalDeviceConfiguration, ConfigValidationError

device_config_path = Path(get_package_share_directory("alex_bringup")) / "params" / "device_configuration.yaml"

def load_nodes_from_config(ld: LaunchDescription):
    
    launch_logger = logging.get_logger('alex_controller.launch')

    try:
        # Load the device configuration
        configuration = ExternalDeviceConfiguration.parse_from_yaml_path(device_config_path)
    except ConfigValidationError as e:
        launch_logger.error(f"Configuration validation error: {e}")
        return
    
    for device_name, device in configuration.devices.items():
        if not device.has_controller():
            continue
        launch_logger.info(f"Device: {device_name} | Node: {device.controller.node_name}")

        namespace = f"{device_name}/" + device.controller.construct_namespace_prefix()
        namespace = namespace.replace("-","_")
        _node = Node(
            package="alex_controller",
            executable=device.controller.node_name,
            name=device.controller.node_name,
            namespace=namespace,
            parameters=[device.controller.parameters, {"device_id": device_name}],
        )
        ld.add_action(_node)

def generate_launch_description():

    ld = LaunchDescription()

    logic_controller_node = Node(
        package="alex_controller",
        executable="logic_controller_node",
        name="logic_controller_node",
        parameters=[{"external_device_config_path": str(device_config_path)}]
    )

    raven_parser = Node(
        package="alex_controller",
        executable="raven_parser",
        name="raven_parser",
    )

    alex_mode_regulator = Node(
        package="alex_controller",
        executable="alex_mode_regulator",
        name="alex_mode_regulator",
    )
    
    activation_action = ActivatingNodeAction(ld,
        package_name="alex_controller",
    )

    load_nodes_from_config(ld)

    ld.add_action(logic_controller_node)
    ld.add_action(raven_parser)
    ld.add_action(alex_mode_regulator)
    ld.add_action(activation_action)

    return ld
