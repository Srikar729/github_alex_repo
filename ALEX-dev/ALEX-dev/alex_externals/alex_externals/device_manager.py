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
from rclpy.node import Node
# ROS Interface Import
from std_msgs.msg import String
from alex_interfaces.msg import SerialMessage
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.device_configuration_handler import ExternalDeviceConfiguration
# Python Import
from json import dumps
from pathlib import Path
from typing import Optional
from pydantic import BaseModel
from serial.tools import list_ports

class SerialPortDetails(BaseModel):
    serial_number: Optional[str] = None
    manufacturer: Optional[str] = None
    product: Optional[str] = None
    port: str
    description: Optional[str] = None

class PortStatus(BaseModel):
    port: str
    device_name: Optional[str] = None  # If device_name is None, it's unlisted
    is_available: bool
    is_active: bool = False
    port_details: SerialPortDetails
    conflict: bool = False
    conflict_details: Optional[list[str]] = []

    def __eq__(self, other):
        if not isinstance(other, PortStatus):
            return False
        return all((
            self.port == other.port,
            self.device_name == other.device_name,
            self.is_available == other.is_available,
            self.is_active == other.is_active,
            self.port_details == other.port_details,
            self.conflict == other.conflict,
            self.conflict_details == other.conflict_details
        ))

class DeviceManager(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.parameters()
        self.load_device_config(self.config_path)

        self.device_active_state: dict[str, bool] = {}
        self.previous_device_status: list[PortStatus] = []
        self.create_device_subscribers()

        self.device_status_pub = self.create_publisher(String, "device_status", 5)

        self.device_check_timer = self.create_timer(5.0, self.device_check)
        self.device_check()

        self.logger.info(f"{node_name} is running")
    
    def parameters(self,):
        # declare ros 2 parameters
        parameters = dict(
            external_device_config_path = "/home/cobot/ros2_ws/install/alex_bringup/share/alex_bringup/params/device_configuration.yaml",
        )
        self.declare_parameters("", parameters=parameters.items())
        
        # get ros 2 parameters
        self.config_path:str  = self.get_parameter('external_device_config_path').value

    def load_device_config(self, config_path: Path):
        self.configuration = ExternalDeviceConfiguration.parse_from_yaml_path(config_path)
        self.logger.debug(self.configuration.model_dump_json(indent=True))
    
    def create_device_subscribers(self, ):
        for device_name, device in self.configuration.devices.items():
            for node in device.external_nodes:
                if node.node_name != "serial_interface_lifecycle": continue
                device_namespace = f"{device_name}/{node.construct_namespace_prefix()}"
                serial_topic_name = f"{device_namespace}from_serial".replace("-","_")
                self.create_subscription(SerialMessage, serial_topic_name, self.handle_device_serial(device_namespace), 5)

    def handle_device_serial(self, device_namespace:str):
        def caller(msg):
            self.handle_serial_message(device_namespace, msg)
        return caller        
    
    def handle_serial_message(self, device_namespace: str, message: SerialMessage):
        self.logger.debug(f"Got message from: {device_namespace}: {message}")
        self.device_active_state[device_namespace] = bool(message.state)

    @staticmethod
    def is_device_port(input_str: str) -> bool:
        return input_str.startswith("/dev/")

    def get_connected_ports(self) -> list[SerialPortDetails]:
        # Fetch the list of serial ports
        ports = list_ports.comports()
        serial_ports = []

        # Process each port and store relevant details in SerialPortDetails
        for port in ports:
            if port.device.startswith('/dev/ttyS'): continue
            serial_port_info = SerialPortDetails(
                port=port.device,
                description=port.description,
                manufacturer=port.manufacturer,
                product=port.product,
                serial_number=port.serial_number
            )
            serial_ports.append(serial_port_info)
        
        return serial_ports

    def get_external_devices(self, ):
        external_devices: dict[str, str] = {}
        for device_name, device in self.configuration.devices.items():
            for node in device.external_nodes:
                serial_port_keys = ["serial_number", "device_port", "ph_meter_serial_number"]
                if not any(key in serial_port_keys for key in  node.parameters.keys()):
                    continue
                name = f"{device_name}{node.construct_namespace_prefix()}"
                serial_number = node.parameters.get("serial_number")
                device_port   = node.parameters.get("device_port")
                ph_meter_serial_number = node.parameters.get("ph_meter_serial_number")
                external_devices[name] = serial_number or device_port or ph_meter_serial_number
        
        return external_devices

    def get_ports_with_device_status(self, external_devices: dict[str, str], connected_ports: list[SerialPortDetails]) -> list[PortStatus]:
        port_statuses: list[PortStatus] = []
        port_map = {}  # Track port usage to detect conflicts
        
        # Iterate through the external devices
        for device_name, serial_number_or_port in external_devices.items():
            is_available = False
            device_port = None
            conflict = False
            conflict_details = []

            serial_number = device_port = None
            
            # Check if the value is a serial number or port
            if self.is_device_port(serial_number_or_port):
                # Treat as a device port
                device_port = serial_number_or_port
                # Check if the port is already used
                if device_port in port_map:
                    conflict = True
                    conflict_details.append(port_map[device_port])
                # Check for matching port in the connected ports
                for port in connected_ports:
                    if port.port == device_port:
                        is_available = True
                        port_map[device_port] = device_name  # Map port to device
                        break
            else:
                # Treat as serial number
                serial_number = serial_number_or_port
                # Check for matching serial number in the connected ports
                for port in connected_ports:
                    if port.serial_number == serial_number:
                        is_available = True
                        device_port = port.port
                        # Check for conflicts in the port_map
                        if device_port in port_map:
                            conflict = True
                            conflict_details.append(port_map[device_port])
                        port_map[device_port] = device_name  # Map port to device
                        break
            
            # Create the port status info for each port
            device_port = device_port or "Unknown"
            port_status = PortStatus(
                port=device_port,
                device_name=device_name,
                is_available=is_available,
                is_active=self.device_active_state.get(device_name, False),
                port_details=next((port for port in connected_ports if port.port == device_port), SerialPortDetails(port=device_port, serial_number=serial_number)),
                conflict=conflict,
                conflict_details=conflict_details if conflict else None,
            )

            port_statuses.append(port_status)

        # Now, we need to handle cases where there are connected devices not listed in external_devices
        for connected_port in connected_ports:
            # If this port is not listed in any device and has no conflict, add it as unlisted
            if not any(port_status.port == connected_port.port for port_status in port_statuses):
                port_status = PortStatus(
                    port=connected_port.port,
                    device_name=None,
                    is_available=True,
                    port_details=connected_port,
                    conflict=False,
                    conflict_details=None,
                )
                port_statuses.append(port_status)
        
        return port_statuses
    
    @staticmethod
    def compare_device_status(device_status: list[PortStatus], previous_device_status: list[PortStatus]) -> bool:
        # Convert each PortStatus object to a dictionary
        device_status_dicts = [port.model_dump(mode="python") for port in device_status]
        previous_device_status_dicts = [port.model_dump(mode="python") for port in previous_device_status]
        
        # Sort the dictionaries to ensure the order doesn't matter (optional)
        device_status_dicts.sort(key=lambda x: x['port'])  # Sorting by port name as an example
        previous_device_status_dicts.sort(key=lambda x: x['port'])
        
        # Compare the two lists of dictionaries
        return device_status_dicts == previous_device_status_dicts

    def device_check(self):
        self.logger.debug("Checking device status")
        connected_ports = self.get_connected_ports()
        external_devices = self.get_external_devices()

        device_status = self.get_ports_with_device_status(external_devices, connected_ports)
        if self.compare_device_status(device_status, self.previous_device_status):
            self.logger.debug("No new updates")
            return
        self.previous_device_status = device_status

        self.logger.info("Got updated device status")
        device_status_msg = String()
        device_status_msg.data = dumps([status.model_dump() for status in device_status])
        self.device_status_pub.publish(device_status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeviceManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
