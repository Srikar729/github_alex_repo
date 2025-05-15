import yaml 
from enum import Enum
from pathlib import Path
from typing import Dict, Any, Optional
from pydantic import BaseModel, ConfigDict, ValidationError, field_validator, model_validator
from pydantic_core import ErrorDetails
from alex_utilities.common_utilities import list_executables, get_device_namespace
from rclpy.client import Client
from rclpy.subscription import Subscription

global_model_config = ConfigDict(extra='forbid')

class DeviceType(Enum):
    LIQUID_DISPENSOR = 'liquid-dispensor'
    PH_METER = 'pH-meter'
    MICROBALANCE = 'weight-balance'
    MAGNETIC_STIRRER = 'magnetic-stirrer'
    HOMOGENIZER = 'homogenizer'
    CENTRIFUGE = "centrifuge"
    SONICATOR = 'sonicator'
    ZED_CAMERA = 'zed-camera'
    ROBOT = 'robot'

class LiquidDispensorRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    level_service: Client      = None
    dosing_service: Client     = None
    volume_topic: Subscription = None
    dosing_topic: Subscription = None

    def get_level_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        level_service_name  = f"/{device_namespace}/start_liquid_level_reading"
        return level_service_name

    def set_level_service(self, client: Client):
        self.level_service = client
    
    def get_dosing_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        dosing_service_name = f"/{device_namespace}/start_liquid_dosing"
        return dosing_service_name
    
    def set_dosing_service(self, client: Client):
        self.dosing_service = client

    def get_volume_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        volume_topic_name   = f"/{device_namespace}/liquid_volume"
        return volume_topic_name
    
    def set_volume_topic(self, subscriber: Subscription):
        self.volume_topic = subscriber
    
    def get_dosing_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        dosing_topic_name   = f"/{device_namespace}/liquid_dosing_status"
        return dosing_topic_name
    
    def set_dosing_topic(self, subscriber: Subscription):
        self.dosing_topic = subscriber

class PHMeterRosDetails(BaseModel):
    model_config                         = ConfigDict(arbitrary_types_allowed=True)
    ph_service: Client                   = None
    temperature_service: Client          = None
    ph_read_topic: Subscription          = None
    temperature_read_topic: Subscription = None
    controller_service: Client           = None

    def get_ph_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name = f"/{device_namespace}/ph/start_ph_reading"
        return service_name

    def set_ph_service(self, client: Client):
        self.ph_service = client
    
    def get_temperature_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name = f"/{device_namespace}/temperature/start_temperature_reading"
        return service_name

    def set_temperature_service(self, client: Client):
        self.temperature_service = client
    
    def get_ph_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name   = f"/{device_namespace}/ph/ph_reading"
        return topic_name
    
    def set_ph_read_topic(self, subscriber: Subscription):
        self.ph_read_topic = subscriber
    
    def get_temperature_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name   = f"/{device_namespace}/temperature/temperature_reading"
        return topic_name
    
    def set_temperature_read_topic(self, subscriber: Subscription):
        self.temperature_read_topic = subscriber
    
    def get_controller_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name = f"/{device_namespace}/measure_ph_value"
        return service_name

    def set_controller_service(self, client: Client):
        self.controller_service = client

class MicrobalanceRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    start_service: Client      = None
    read_topic: Subscription   = None
    controller_service: Client = None

    def get_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name = f"/{device_namespace}/weight_balance/start_weight_balance"
        return service_name

    def set_start_service(self, client: Client):
        self.start_service = client
    
    def get_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name   = f"/{device_namespace}/weight_balance/weight_data"
        return topic_name

    def set_read_topic(self, subscriber: Subscription):
        self.read_topic = subscriber

    def get_controller_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name = f"/{device_namespace}/measure_weight_balance"
        return service_name

    def set_controller_service(self, client: Client):
        self.controller_service = client

class MagneticStirrerRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    start_service: Client    = None
    read_topic: Subscription = None

    def get_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name   = f"/{device_namespace}/start_magnetic_stirrer"
        return service_name

    def set_start_service(self, client: Client):
        self.start_service = client
    
    def get_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name   = f"/{device_namespace}/magnetic_stirrer_status"
        return topic_name
    
    def set_read_topic(self, subscriber: Subscription):
        self.read_topic = subscriber

class HomogenizerRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    start_service: Client    = None
    read_topic: Subscription = None

    def get_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name   = f"/{device_namespace}/start_homogenizer"
        return service_name

    def set_start_service(self, client: Client):
        self.start_service = client
    
    def get_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name   = f"/{device_namespace}/homogenizer_status"
        return topic_name
    
    def set_read_topic(self, subscriber: Subscription):
        self.read_topic = subscriber


class CentrifugeRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    start_service: Client = None
    read_topic: Subscription = None
    
    def get_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name     = f"/{device_namespace}/start_centrifuge"
        return service_name
    
    def set_start_service(self, client: Client):
        self.start_service = client

    def get_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name       = f"/{device_namespace}/centrifuge_status"
        return topic_name

    def set_read_topic(self, subscriber: Subscription):
        self.read_topic = subscriber
    
class SonicatorRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    start_service: Client    = None
    start_heater: Client     = None
    read_topic: Subscription = None

    def get_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name   = f"/{device_namespace}/start_sonication"
        return service_name

    def set_start_service(self, client: Client):
        self.start_service = client
    
    def get_heater_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name   = f"/{device_namespace}/start_heating"
        return service_name

    def set_start_heater(self, client: Client):
        self.start_heater = client
    
    def get_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name       = f"/{device_namespace}/sonicator_status"
        return topic_name
    
    def set_read_topic(self, subscriber: Subscription):
        self.read_topic = subscriber

class ZedCameraRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    left_camera: Subscription     = None
    right_camera: Subscription     = None

    def get_left_camera_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name       = f"/{device_namespace}/left_camera"
        return topic_name
    
    def set_left_camera_topic_name(self, subscriber: Subscription):
        self.left_camera = subscriber

    def get_right_camera_topic_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        topic_name       = f"/{device_namespace}/right_camera"
        return topic_name
    
    def set_right_camera_topic_name(self, subscriber: Subscription):
        self.right_camera = subscriber

class RobotRosDetails(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    move_service: Client = None
    gripper_service: Client = None
    controller_service: Client = None

    def get_move_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name     = f"/{device_namespace}/move_robot"
        return service_name
    
    def set_move_service(self, client: Client):
        self.move_service = client
    
    def get_gripper_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name     = f"/{device_namespace}/gripper_control"
        return service_name
    
    def set_gripper_service(self, client: Client):
        self.gripper_service = client
    
    def get_controller_service_name(self, device_name: str):
        device_namespace = get_device_namespace(device_name)
        service_name     = f"/{device_namespace}/arm_control"
        return service_name
    
    def set_controller_service(self, client: Client):
        self.controller_service = client

RosDetailsType = (PHMeterRosDetails | LiquidDispensorRosDetails | MicrobalanceRosDetails | 
                  MagneticStirrerRosDetails | CentrifugeRosDetails | SonicatorRosDetails | 
                  ZedCameraRosDetails | RobotRosDetails | HomogenizerRosDetails)

class ExternalNode(BaseModel):
    model_config = global_model_config
    node_name: str
    prefix: Optional[str] = ""
    parameters: Dict[str, Any] = {}

    def construct_namespace_prefix(self,):
        """Construct namespace with trailing slash"""
        return f"{self.prefix}/" if self.prefix else ""

class DeviceControl(ExternalNode):
    model_config = global_model_config
    node_name: str

class DeviceConfig(BaseModel):
    model_config = global_model_config
    device_type: DeviceType
    external_nodes: list[ExternalNode]
    controller: Optional[DeviceControl]= None
    ros_details: RosDetailsType

    def get_external_node_names(self,):
        return list( node.node_name for node in self.external_nodes)
    
    def has_controller(self,):
        return self.controller is not None

    @model_validator(mode='before')
    @classmethod
    def set_ros_details(cls, values: dict):
        ros_details = {
            DeviceType.PH_METER.value : PHMeterRosDetails, 
            DeviceType.LIQUID_DISPENSOR.value : LiquidDispensorRosDetails, 
            DeviceType.MICROBALANCE.value : MicrobalanceRosDetails, 
            DeviceType.MAGNETIC_STIRRER.value : MagneticStirrerRosDetails, 
            DeviceType.HOMOGENIZER.value : HomogenizerRosDetails, 
            DeviceType.CENTRIFUGE.value : CentrifugeRosDetails, 
            DeviceType.SONICATOR.value : SonicatorRosDetails, 
            DeviceType.ZED_CAMERA.value : ZedCameraRosDetails, 
            DeviceType.ROBOT.value : RobotRosDetails, 
        }
        device_type = values.get("device_type")
        if device_type not in ros_details.keys():
            raise ValueError(f"Unknown device type: {device_type}")
        values["ros_details"] = ros_details.get(device_type)()
        return values
        
    @field_validator("external_nodes")
    @classmethod
    def validate_external_nodes_names(cls, value: list[ExternalNode]):
        """Make sure the given nodes are available"""
        node_names = set( node.node_name for node in value )
        available_values = list_executables('alex_externals')
        invalid_keys = node_names - available_values
        if invalid_keys:
            raise ValueError(f"Invalid node_name in external_nodes: {invalid_keys}")
        return value
    
    @field_validator("controller")
    @classmethod
    def validate_controller_nodes_keys(cls, value: Optional[DeviceControl]= None):
        """Make sure the given nodes are available"""
        if value is None:
            return value
        node_names = value.node_name
        available_values = list_executables('alex_controller')
        if node_names not in available_values:
            raise ValueError(f"Invalid node_name in controller_node: {node_names}")
        return value

class ExternalDevicesConfig(BaseModel):
    model_config = global_model_config
    devices: Dict[str, DeviceConfig]
    
    def get_all_devices_names(self):
        return list(self.devices.keys())

class ConfigValidationError(Exception):
    def __init__(self, errors:list[ErrorDetails] ) -> None:
        formated_errors = [ f'{error.get("type")}:: {error.get("msg")} : trace={error.get("loc")} : input={error.get("input")}'
                           for error in errors ]

        super().__init__(*formated_errors)

class ExternalDeviceConfiguration(ExternalDevicesConfig):

    @classmethod
    def parse_from_yaml_path(cls, config_path: str|Path):
        if isinstance(config_path, str):
            config_path = Path(config_path)
        if not isinstance(config_path, Path):
            raise
        if not config_path.exists():
            raise FileNotFoundError("Verify the config path")
        if not config_path.is_file():
            raise FileNotFoundError("Path is not a file to read")
        config_text = config_path.read_text()
        _config = yaml.safe_load(config_text)
        return cls._validate_data(_config)
    
    @classmethod
    def parse_from_dict(cls, data: dict):
        return cls._validate_data(data)

    @classmethod
    def parse_from_json(cls, data: str):
        try:
            device_config = ExternalDevicesConfig.model_validate_json(data)
        except ValidationError as e:
            errors = e.errors(include_url=False)
            raise ConfigValidationError(errors)
        return device_config

    @staticmethod
    def _validate_data(data:dict):
        try:
            device_config = ExternalDevicesConfig.model_validate(data)
        except ValidationError as e:
            errors = e.errors(include_url=False)
            raise ConfigValidationError(errors)
        return device_config
