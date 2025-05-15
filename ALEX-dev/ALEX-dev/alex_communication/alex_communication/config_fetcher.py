"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 

"""
# ROS Import
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.task import Future
# ROS Interface Import
from alex_interfaces.srv import MqttAttributes
# Python Import
import json
import yaml
from pathlib import Path
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.device_configuration_handler import ExternalDeviceConfiguration

class ConfigFetcher(LifecycleNode):
    """Upon Configuring the lifecycle. Fetch the config and write to a file"""
    def __init__(self):
        self.node_name = change_case(self.__class__.__name__)
        super().__init__(self.node_name)
        self.logger = self.get_logger()

        # Define a device configuration name
        self.device_config = "device_config"
        self.mqtt_attributes_srv = None

        #TODO Should come from a parameters as ALEX store
        self.device_config_path = Path.home() / "ros2_ws" / "install" / "alex_bringup" / "share" / "alex_bringup" / "params" / "device_configuration.yaml"

    def on_configure(self, state):
        """Called when the node enters the "configured" state."""
        self.logger.info(f"{self.node_name} node is being configured.")
        
        # Initialize the MQTT service client
        self.mqtt_attributes_srv = self.create_client(MqttAttributes, "mqtt_attributes")
        is_server_ready = self.mqtt_attributes_srv.wait_for_service(10)
        if not is_server_ready:
            self.logger.error("The mqtt attribute server is not ready even after waiting.. ")
            raise Exception("DIE")
        self.logger.info("MqttAttribute server is ready to go")

        # Fetch and write the configuration once the node is configured
        self.get_all_device_parameter()
        
        self.logger.info("Configuration Successfully done")
        return TransitionCallbackReturn.SUCCESS

    def handle_mqtt_parmeter(self, future:Future):
        self.logger.info("Got all device parameter")
        response: MqttAttributes.Response = future.result()
        if not response.success:
            self.logger.error(f"Mqtt Attribute fetching failed with error: {response.reason}")
            return None
        
        attributes = json.loads(response.attributes)
        data = {"devices": attributes}
        design_configuration = ExternalDeviceConfiguration.parse_from_dict(data)

        if design_configuration:
            # Write the device configuration to a file
            self.write_device_config(self.device_config_path, design_configuration)
        
        self.trigger_shutdown()
        return design_configuration
        
    def get_all_device_parameter(self):
        """Fetch all device parameters using the MQTT service."""
        self.logger.info("Getting all device parameter")
        request = MqttAttributes.Request()
        request.device_id = MqttAttributes.Request.DEVICE_ALL
        request.attribute_key = self.device_config
        future = self.mqtt_attributes_srv.call_async(request)
        future.add_done_callback(self.handle_mqtt_parmeter)

    def write_device_config(self, path: Path, config: ExternalDeviceConfiguration):
        """Write the device configuration to a file asynchronously."""
        exclude_keys = {
            'devices': {
                "__all__": "ros_details"
            }
        }
        data = config.model_dump(mode="json", exclude_defaults = True, exclude_none = True, exclude_unset = True, exclude=exclude_keys)
        try:
            with path.open("w") as file:
                yaml.dump(data, file, default_flow_style=False)
            self.logger.info("Device configuration written to file.")
            return True
        except Exception as e:
            self.logger.error(f"Error writing device config to {path}: {e}")
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ConfigFetcher()
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
