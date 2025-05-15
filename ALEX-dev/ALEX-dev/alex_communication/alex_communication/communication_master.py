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
from rclpy.task import Future
from rclpy.client import Client
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
# ROS Interface Import
from std_msgs.msg import String
from alex_interfaces.msg import (
    ActionFileInterface, ActionFileInterfaceReference, CobotAction, CobotActionOption, CobotUpdates,
    PHMeterStatus, TemperatureStatus, Weight, ModeStatus, SocketMessage, MqttMessage, LiquidVolumeStatus, LiquidDosingStatus,
    ActiveNodesStatus, MagneticStirrer, CentrifugeStatus, Sonicator,
)
from alex_interfaces.srv import (
    PHMeterCommand, TemperatureCommand, WeightCommand, MagneticStirrerCommand, PauseResumeAction, LiquidDoserCommand,
    LiquidLevelCommand, CentrifugeCommand, ControlMode, SonicatorCommand,
)
from alex_interfaces.action import ExecuteActionFile
# Python Import
import json
import time
# Utilities Import
from alex_utilities.common_utilities import change_case, rosmsg_to_dict
from alex_utilities.device_configuration_handler import ExternalDeviceConfiguration, ConfigValidationError, DeviceType, RosDetailsType

class CommunicationMaster(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.parameters()
        self.set_subscribers()
        self.set_publishers()
        self.set_actions()
        self.set_services()
        
        self.external_devices = self.load_external_devices(self.external_device_config_path)
        self.logger.info(f"{node_name} is running")
    
    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('external_device_config_path', value='/home/dexus/ros2_ws/install/alex_bringup/share/alex_bringup/params/device_configuration.yaml',   descriptor=ParameterDescriptor(description="Path to the devices config file"))
        # get ros 2 parameters
        self.external_device_config_path:str   = self.get_parameter('external_device_config_path').value

    def set_subscribers(self,):
        self.sub_socket_msg = self.create_subscription(SocketMessage,     "socket_receiver",      self.socket_handler,            10)
        self.sub_alex_mode_msg = self.create_subscription(ModeStatus,     "mode_publisher",       self.mode_handler,              10)
        self.sub_mqtt_msg   = self.create_subscription(MqttMessage,       "mqtt_receiver",        self.mqtt_handler,              10)
        self.sub_node_status= self.create_subscription(ActiveNodesStatus, "active_node_status",   self.node_status_handler,       10)
        self.sub_device_status= self.create_subscription(String,          "device_status",        self.device_status_handler,     10)

    def set_publishers(self,):
        self.pub_socket_msg = self.create_publisher(SocketMessage, "socket_sender", 10)
        self.pub_raven_msg  = self.create_publisher(String, 'raven_msg', 10)
        self.pub_mqtt_msg   = self.create_publisher(MqttMessage, "mqtt_sender", 10)
    
    def set_actions(self,):
        self.afi_action_client = ActionClient(self, ExecuteActionFile, "execute_afi")
    
    def set_services(self,):
        self.manage_pause_resume_srv        = self.create_client(PauseResumeAction, "manage_pause_resume")
        self.alex_mode_control_srv          = self.create_client(ControlMode, "alex_mode_regulator")
    
    # MARK: External Devices
    def load_external_devices(self, device_config_path: str ):
        try:
            configuration = ExternalDeviceConfiguration.parse_from_yaml_path(device_config_path)
        except ConfigValidationError as e:
            self.logger.error(f"Unable to parse external device: {e}")
            return

        external_devices: dict[DeviceType, dict[str, RosDetailsType]] = {}
        
        for device_name, device_config in configuration.devices.items():
            device_ros_details = device_config.ros_details
            if device_config.device_type not in external_devices:
                external_devices[device_config.device_type] = {}
            # Creating
            if device_config.device_type == DeviceType.PH_METER:
                # MARK: __PH
                service_name  = device_ros_details.get_ph_service_name(device_name)
                ph_service = self.create_client(PHMeterCommand, service_name)
                device_ros_details.set_ph_service(ph_service)

                service_name  = device_ros_details.get_temperature_service_name(device_name)
                temperature_service = self.create_client(TemperatureCommand, service_name)
                device_ros_details.set_temperature_service(temperature_service)

                ph_topic_name = device_ros_details.get_ph_topic_name(device_name)
                ph_read_topic = self.create_subscription(PHMeterStatus, ph_topic_name, self.topic_wrapper(device_name, self.send_ph_meter_reading), 10)
                device_ros_details.set_ph_read_topic(ph_read_topic)

                temperature_topic_name = device_ros_details.get_temperature_topic_name(device_name)
                temperature_read_topic = self.create_subscription(TemperatureStatus, temperature_topic_name, self.topic_wrapper(device_name, self.send_ph_meter_reading), 10)
                device_ros_details.set_ph_read_topic(temperature_read_topic)

            elif device_config.device_type == DeviceType.LIQUID_DISPENSOR:
                # MARK: __Liquid Dispensor
                level_service_name  = device_ros_details.get_level_service_name(device_name)
                level_service       = self.create_client(LiquidLevelCommand, level_service_name)
                device_ros_details.set_level_service(level_service)

                dosing_service_name = device_ros_details.get_dosing_service_name(device_name)
                dosing_service      = self.create_client(LiquidDoserCommand, dosing_service_name)
                device_ros_details.set_dosing_service(dosing_service)

                volume_topic_name = device_ros_details.get_volume_topic_name(device_name)
                volume_topic      = self.create_subscription(LiquidVolumeStatus, volume_topic_name, self.topic_wrapper(device_name, self.send_liquid_level_data), 10)
                device_ros_details.set_volume_topic(volume_topic)

                dosing_topic_name = device_ros_details.get_dosing_topic_name(device_name)
                dosing_topic      = self.create_subscription(LiquidDosingStatus, dosing_topic_name, self.topic_wrapper(device_name, self.send_liquid_dosing_status), 10)
                device_ros_details.set_dosing_topic(dosing_topic)
                
            elif device_config.device_type == DeviceType.MICROBALANCE:
                # MARK: __Microbalance
                service_name  = device_ros_details.get_service_name(device_name)
                start_service =  self.create_client(WeightCommand, service_name)
                device_ros_details.set_start_service(start_service)

                topic_name = device_ros_details.get_topic_name(device_name)
                # BUG: need to add prefix
                read_topic = self.create_subscription(Weight, topic_name, self.topic_wrapper(device_name, self.send_weight_balance_status), 10)
                device_ros_details.set_read_topic(read_topic)

            elif device_config.device_type == DeviceType.MAGNETIC_STIRRER:
                # MARK: __Magnetic Stirrer
                service_name  = device_ros_details.get_service_name(device_name)
                start_service = self.create_client(MagneticStirrerCommand, service_name)
                device_ros_details.set_start_service(start_service)

                topic_name = device_ros_details.get_topic_name(device_name)
                read_topic = self.create_subscription(MagneticStirrer, topic_name, self.topic_wrapper(device_name, self.send_magnetic_stirrer_status), 10)
                device_ros_details.set_read_topic(read_topic)
            
            elif device_config.device_type == DeviceType.CENTRIFUGE:
                # MARK: __Centrifuge
                service_name = device_ros_details.get_service_name(device_name)
                start_service = self.create_client(CentrifugeCommand, service_name)
                device_ros_details.set_start_service(start_service)

                topic_name = device_ros_details.get_topic_name(device_name)
                read_topic = self.create_subscription(CentrifugeStatus, topic_name, self.topic_wrapper(device_name, self.send_centrifuge_status), 10)
                device_ros_details.set_read_topic(read_topic)
                
            elif device_config.device_type == DeviceType.SONICATOR:
                # MARK: __Sonicator
                service_name  = device_ros_details.get_service_name(device_name)
                start_service = self.create_client(SonicatorCommand, service_name)
                device_ros_details.set_start_service(start_service)

                heater_name = device_ros_details.get_heater_name(device_name)
                start_service = self.create_client(SonicatorCommand, heater_name)
                device_ros_details.set_start_heater(start_service)

                topic_name = device_ros_details.get_topic_name(device_name)
                read_topic = self.create_subscription(Sonicator, topic_name, self.topic_wrapper(device_name, self.send_sonicator_status), 10)
                device_ros_details.set_read_topic(read_topic)

            external_devices[device_config.device_type].update({ device_name: device_ros_details })
        return external_devices
    
    def topic_wrapper(self, device_name, func):
        def caller(data):
            response = func(device_name, data)
            return response
        return caller

    def socket_handler(self, data:SocketMessage):
        self.logger.info(f"Got `socket_handler` message: {data.channel=} {data.msg=}")
        if data.channel == "":
            self.logger.warning("Got socket_handler without channel")
        elif data.channel == "execute_afi":
            self.execute_afi(data.msg)
        elif data.channel == "manage_pause_resume":
            self.manage_pause_resume(data.msg)
        elif data.channel == "cmd_ph":
            self.handle_ph_control(data.msg)
        elif data.channel == "cmd_container":
            self.handle_liquid_level_control(data.msg)
        elif data.channel == "cmd_liquid_doser":
            self.handle_liquid_doser(data.msg)
        elif data.channel == "cmd_weight_balance":
            self.handle_weight_balance(data.msg)
        elif data.channel == "cmd_raven":
            self.handle_raven_msg(data.msg)
        elif data.channel == "cmd_magnetic_stirrer":
            self.handle_magnetic_stir(data.msg)
        elif data.channel == "cmd_control_mode":
            self.handle_control_mode(data.msg)
        elif data.channel == "cmd_centrifuge":
            self.handle_centrifuge(data.msg)
        elif data.channel == "cmd_sonicator":
            self.handle_sonicator(data.msg)
        else: 
            self.logger.warning("Invalid Channel from socket channel")

    def mqtt_handler(self, data:MqttMessage):
        self.logger.info(f"Got `mqtt_handler` message: {data.device_id=} {data.msg=}")
    
    def node_status_handler(self, msg:ActiveNodesStatus):
        self.logger.debug(f"Got Node Status Handler, {msg}")
        
        data = rosmsg_to_dict(msg)
        data["ts"] = round(time.time() * 1000),
        
        self.send_robot_telemetry("node_status", data)
    
    def device_status_handler(self, msg: String):
        self.logger.debug(f"Got device_status: {msg.data}")

        self.send_robot_telemetry("device_status", msg.data)
    
    # MARK: Control Mode
    def handle_control_mode(self, data: String):
        json_data = json.loads(data)
        """
        json_data = {
            mode: raven/afi/
        }
        """
        control_modes = {
            "afi": ControlMode.Request.MODE_AFI,
            "raven": ControlMode.Request.MODE_RAVEN,
        }
        mode = json_data.get("mode", None)
        
        request = ControlMode.Request()
        request.mode = control_modes.get(mode, ControlMode.Request.MODE_NONE)
        request.origin = ControlMode.Request.ORIGIN_URP

        future = self.alex_mode_control_srv.call_async(request)
        future.add_done_callback(self.control_mode_srv_callback)
        # TODO: Should sent response?
    
    def control_mode_srv_callback(self, future: Future):
        result: ControlMode.Response = future.result()
        if not result.success:
            self.logger.info(f"Failed to set Control Mode from URP: {result.reason}")
            return
        self.logger.info("Set Control Mode from URP Successfully")

    def mode_handler(self, data: ModeStatus):
        self.logger.info(f"Got `mode` message: {data.success}")
        device_id = "key-device"
        data = dict(
            ts = round(time.time() * 1000),
            success      = data.success,
            reason       = data.reason,
            origin       = data.origin,
            current_mode = data.current_mode,
        )
        socket_message = SocketMessage(channel="execution_mode")
        socket_message.msg = json.dumps(data)
        self.pub_socket_msg.publish(socket_message)
        mqtt_message = MqttMessage(device_id=device_id)
        mqtt_message.msg = json.dumps({"execution_mode" : data})
        self.pub_mqtt_msg.publish(mqtt_message)

    # MARK: AFI ACTION
    def execute_afi(self, data: str):
        try: 
            json_data = json.loads(data)
            action_file_data = json_data['afi']
            job_id = json_data["jobId"]
            afi = ActionFileInterface()
            afi.version = float(action_file_data["metadata"].get("compilerVersion"))
            afi.author  = action_file_data["metadata"].get("author")
            afi.automation_target = action_file_data["metadata"].get("automationTarget")
            # Parsing reference_objects
            reference_objects:list[ActionFileInterfaceReference] = list()
            object_references = action_file_data["metadata"].get("objectReferences", dict()) # If value is not there emtpy dict
            for key, value in object_references.items():
                reference_objects.append(ActionFileInterfaceReference(reference=key, object=value))
            if reference_objects:
                afi.object_references = reference_objects or [ActionFileInterfaceReference()]
            # Parsing actions
            actions:list[CobotAction] = []
            for action in action_file_data.get("actions", actions): # If value is not there emtpy list
                action_options:dict = action.pop("option", {})
                action_quantity = action.pop("quantity", {})
                action_parameter = action.pop("parameter", "")
                cobot_action = CobotAction(**action)
                cobot_action.parameter = str(action_parameter)
                if action_quantity:
                    cobot_action.quantity.unit = action_quantity["unit"]
                    cobot_action.quantity.value = float(action_quantity["value"])
                # Parsing CobotActionOptions
                cobot_action_options = []
                for key, value in action_options.items():
                    cobot_action_options.append(
                        CobotActionOption(key=key, value=str(value))
                    )
                cobot_action.options = cobot_action_options
                actions.append(cobot_action)
            if actions:
                afi.actions = actions
        except json.JSONDecodeError as error:
            self.logger.warning(f"Failed to load json with {str(error)=}")
            self.send_afi_error_state("Invalid json format")
            return
        except KeyError as error:
            self.logger.warning(f"Invalid key in AFI, {error=}")
            self.send_afi_error_state("Invalid Key, Failed to parse action file")
            return
        except AssertionError as error:
            self.logger.warning(f"Invalid key in AFI, {error=}")
            self.send_afi_error_state(f"Parsing key error: {error}")
            return
        # call the AFI action
        action_goal = ExecuteActionFile.Goal()
        action_goal.job_info = job_id
        action_goal.afi = afi
        action_future = self.afi_action_client.send_goal_async(
            action_goal,
            feedback_callback=self.afi_feedback_callback
        )
        action_future.add_done_callback(self.afi_goal_response_callback)
    
    def afi_feedback_callback(self, goal_handle):
        self.send_afi_feedback_telemetry(goal_handle.feedback)
    
    def afi_goal_response_callback(self, future:Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.warning('AFI Goal rejected :(')
            self.send_afi_error_state("AFI Action rejected, maybe previous action is still in progress")
            return
        result_data = goal_handle.get_result_async()
        result_data.add_done_callback(self.afi_get_result_callback)
    
    def afi_get_result_callback(self, future):
        result: ExecuteActionFile.Result = future.result().result
        if not result.success:
            self.logger.info(f'AFI Goal Failed! with reason: {result.reason}')
        self.send_afi_feedback_telemetry(result)

    def send_afi_error_state(self, error_msg: str) -> None:
        error_result = ExecuteActionFile.Result()
        error_result.reason = error_msg
        error_result.status.total_steps = 0
        error_result.status.current_step_index = 0
        error_result.status.status = CobotUpdates.ERROR
        self.send_afi_feedback_telemetry(error_result)

    def send_afi_feedback_telemetry(self, feedback: ExecuteActionFile.Feedback | ExecuteActionFile.Result) -> None:
        formatted_telemetry = dict(
            ts          = round(time.time()*1000),
            ts_start    = feedback.status.start_timestamp,
            ts_end      = feedback.status.end_timestamp,
            current_step= feedback.status.current_step_index,
            job_id      = feedback.status.job_id,
            status      = feedback.status.status,
            total_steps = feedback.status.total_steps,
            success     = None,
            reason      = None,
        )
        if isinstance(feedback, ExecuteActionFile.Result):
            result_status = dict(
                success = feedback.success,
                reason  = feedback.reason
            )
            formatted_telemetry.update(result_status)

        self.send_robot_telemetry('afi_telemetry', formatted_telemetry)
        
    def send_robot_telemetry(self, key: str, data: str | dict) -> None:
        device_id = "key-device"
        socket_message = SocketMessage(channel=key)
        socket_message.msg = json.dumps(data)
        self.pub_socket_msg.publish(socket_message)
        mqtt_message = MqttMessage(device_id=device_id)
        mqtt_message.msg = json.dumps({key: data })
        self.pub_mqtt_msg.publish(mqtt_message)

    def manage_pause_resume(self, data:str):
        self.logger.info(f"Got manage_pause_resume: {data}")
        data = data.replace('"', "")
        request = PauseResumeAction.Request()
        lookup_dict = {
            "STOP"    : PauseResumeAction.Request.STOP,
            "RESUME"  : PauseResumeAction.Request.RESUME,
            "PAUSE"   : PauseResumeAction.Request.PAUSE,
            "GETMODE" : PauseResumeAction.Request.GETMODE,
        }
        request.mode = lookup_dict.get(data, PauseResumeAction.Request.RESUME)
        self.manage_pause_resume_srv.call_async(request)
    # -----------------AFI Action: END-----------------

    # MARK: PH
    def handle_ph_control(self, data:str):
        """
        json_data = {
            device_id: str
            command: measure/stop
            optional: Optional[str]
            value: float
            job_id: Optional[str]
        }
        """
        json_data = json.loads(data)
        device_id = json_data.get("device_id", "")
        job_id = json_data.get("job_id", "")
        ph_meter_devices = self.external_devices[DeviceType.PH_METER]
        if device_id not in ph_meter_devices:
            self.send_ph_meter_telemetry(
                device_id = device_id,
                status    = PHMeterStatus.ERROR,
                success   = False,
                reason    = "Invalid Device ID"
            )
            return
        ph_service: Client = ph_meter_devices[device_id].ph_service
        temperature_service: Client = ph_meter_devices[device_id].temperature_service
        ph_request = PHMeterCommand.Request()
        command = json_data.get("command")
        if not isinstance(command, str):
            self.send_ph_meter_telemetry(
                device_id = device_id,
                status    = PHMeterStatus.ERROR,
                success   = False,
                reason    = "value measure have to be a valid string"
            )
            return
        ph_request.command = command
        ph_request.job_id  = job_id
        if not ph_service.service_is_ready():
            self.send_ph_meter_telemetry(
                device_id = device_id,
                status    = PHMeterStatus.ERROR,
                success   = False,
                reason    = "pH Service is not ready"
            )
            return
        future = ph_service.call_async(ph_request)
        future.add_done_callback(lambda future: self.handle_ph_service_callback(device_id, "pH", future))

        if command not in [TemperatureCommand.Request.COMMAND_CUSTOM_MEASURE, TemperatureCommand.Request.COMMAND_CUSTOM_STOP]:
            return

        temperature_request = TemperatureCommand.Request()
        if not temperature_service.service_is_ready():
            self.send_ph_meter_telemetry(
                device_id = device_id,
                status    = TemperatureStatus.ERROR,
                success   = False,
                reason    = "Temperature sensor is not ready"
            )
            return
        temperature_request.command = command
        temperature_request.job_id  = job_id
        future = temperature_service.call_async(temperature_request)
        future.add_done_callback(lambda future: self.handle_ph_service_callback(device_id, "Temperature", future))
    
    def handle_ph_service_callback(self, device_id: str, probe_type: str, future: Future):
        response:PHMeterCommand.Response = future.result()
        self.send_ph_meter_telemetry(
            device_id = device_id, 
            status    = response.status,
            reason    = f"{probe_type} probe:{response.status.reason}",
        )

    def send_ph_meter_reading(self, device_id: str, current_data: PHMeterStatus | TemperatureStatus):
        if isinstance(current_data, PHMeterStatus):
            self.send_ph_meter_telemetry(
                device_id = device_id, 
                status    = current_data.status,
                job_id    = current_data.job_id,
                ph_value     = {
                    "value": round(current_data.value.value, 3),
                    "unit" : current_data.value.unit,
                },
                reason    = current_data.reason
            )
        elif isinstance(current_data, TemperatureStatus):
            self.send_ph_meter_telemetry(
                device_id = device_id, 
                status    = current_data.status,
                job_id    = current_data.job_id,
                temperature_value     = {
                    "value": round(current_data.value.value, 3),
                    "unit" : current_data.value.unit,
                },
                reason    = current_data.reason
            )
    
    def send_ph_meter_telemetry(self, device_id:str, status:str, reason:str="", **kwargs):
        ph_data = dict(
            ts        = round(time.time() * 1000),
            device_id = device_id,
            status    = status,
            reason    = reason,
        )
        if kwargs: ph_data.update(kwargs)
        socket_message = SocketMessage(channel="ph_msg")
        socket_message.msg = json.dumps(ph_data)
        self.pub_socket_msg.publish(socket_message)
        del ph_data["device_id"]
        mqtt_message = MqttMessage(device_id=device_id)
        mqtt_message.msg = json.dumps(ph_data)
        self.pub_mqtt_msg.publish(mqtt_message)
        
    # ---------------pH Control: END---------------

    # MARK: Liquid Level
    def handle_liquid_level_control(self, data:str):
        json_data = json.loads(data)
        request = LiquidLevelCommand.Request()
        device_id:str = json_data.get("device_id", "")
        liquid_devices = self.external_devices[DeviceType.LIQUID_DISPENSOR]
        if device_id not in liquid_devices:
            self.send_liquid_level_telemetry(
                    device_id= device_id,
                    success  = False,
                    reason   = "Invalid device_id",
                    status   = LiquidVolumeStatus.STATUS_ERROR,
                )
            return
        request.command      = json_data["measure"]
        service: Client = liquid_devices[device_id].level_service
        if not service.service_is_ready():
            self.send_liquid_level_telemetry(
                device_id= device_id,
                success  = False,
                reason   = "Service not ready.",
                status   = LiquidVolumeStatus.STATUS_ERROR,
            )
            return
        future = service.call_async(request)

    def send_liquid_level_data(self, device_id: str, data: LiquidVolumeStatus):
        self.send_liquid_level_telemetry(
            device_id   = device_id,
            success     = True,
            reason      = data.reason,
            status      = LiquidVolumeStatus.STATUS_WORKING,
            inventory={
                "unit": data.value.unit,
                "value": round(data.value.value, 2),
            },
        )
    
    def send_liquid_level_telemetry(self, device_id, success:bool, reason:str="", **kwargs):
        liquid_data = dict(
            ts        = round(time.time() * 1000),
            device_id = device_id,
            success   = success,
            reason    = reason,
        )
        if kwargs: liquid_data.update(kwargs)
        socket_message = SocketMessage(channel="container_msg")
        socket_message.msg = json.dumps(liquid_data)
        self.pub_socket_msg.publish(socket_message)
        del liquid_data["device_id"]
        mqtt_message = MqttMessage(device_id=device_id)
        mqtt_message.msg = json.dumps(liquid_data)
        self.pub_mqtt_msg.publish(mqtt_message)
    # -----------Liquid Level Control: END-----------

    # MARK: Liquid Doser 
    def handle_liquid_doser(self, data:str):
        json_data = json.loads(data)
        command_lookup = {
            "DISPENSE" : LiquidDoserCommand.Request.COMMAND_DISPENSE,
            "STOP"     : LiquidDoserCommand.Request.COMMAND_STOP,
            "PAUSE"    : LiquidDoserCommand.Request.COMMAND_PAUSE,
            "RESUME"   : LiquidDoserCommand.Request.COMMAND_PAUSE,
            "CALIBRATE": LiquidDoserCommand.Request.COMMAND_CALIBRATE,
        }
        request = LiquidDoserCommand.Request()
        device_id = json_data.get("device_id", "")
        liquid_devices = self.external_devices[DeviceType.LIQUID_DISPENSOR]
        if device_id not in liquid_devices:
            self.send_liquid_doser_telemetry(
                device_id   = device_id,
                success     = False,
                status      = LiquidDosingStatus.ERROR,
                reason      = "Invalid device_id",
            )
            return
        if (command:=json_data.get("command", None)) not in command_lookup:
            self.send_liquid_doser_telemetry(
                device_id   = device_id,
                success     = False,
                status      = LiquidDosingStatus.ERROR,
                reason      = "Invalid command",
            )
            return
        request.job_id    = json_data.get("job_id", "")
        request.command = command_lookup[command]
        if request.command in [LiquidDoserCommand.Request.COMMAND_DISPENSE, LiquidDoserCommand.Request.COMMAND_CALIBRATE]:
            volume = json_data["volume"]
            request.value = float(volume)
        
        service: Client = liquid_devices[device_id].dosing_service
        if not service.service_is_ready():
            self.send_liquid_doser_telemetry(
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = LiquidDosingStatus.ERROR,
                reason      = "Service is not ready",
            )
            return
        future = service.call_async(request)
        self.send_liquid_doser_telemetry(
                device_id   = device_id,
                job_id      = request.job_id,
                success     = True,
                reason      = "Command send to service",
                status      = LiquidDosingStatus.WORKING,
            )

    def send_liquid_dosing_status(self, device_id: str, data: LiquidDosingStatus):
        self.send_liquid_doser_telemetry(
            device_id   = device_id,
            job_id      = data.job_id,
            success     = True,
            status      = data.status,
            value       = {
                "value": data.value.value,
                "unit" : data.value.unit,
            },
            reason      = data.reason,
        )

    def send_liquid_doser_telemetry(self, device_id:str, success:bool, reason:str="", **kwargs):
        data = dict(
            ts = round(time.time() * 1000),
            device_id = device_id,
            success   = success,
            reason    = reason,
        )
        if kwargs: data.update(kwargs)
        socket_message = SocketMessage(channel="liquid_dosing_msg")
        socket_message.msg = json.dumps(data)
        self.pub_socket_msg.publish(socket_message)
        del data["device_id"]
        mqtt_message = MqttMessage(device_id=device_id)
        mqtt_message.msg = json.dumps(data)
        self.pub_mqtt_msg.publish(mqtt_message)

    # -----------Liquid Doser Control: END-----------

    def publish_telemetry(self, socket_channel:str, device_id:str, success:bool, reason:str="", **kwargs):
        data = dict(
            ts = round(time.time() * 1000),
            device_id = device_id,
            success   = success,
            reason    = reason,
        )
        if kwargs: data.update(kwargs)
        socket_message = SocketMessage(channel=socket_channel)
        socket_message.msg = json.dumps(data)
        self.pub_socket_msg.publish(socket_message)
        del data["device_id"]
        mqtt_message = MqttMessage(device_id=device_id)
        mqtt_message.msg = json.dumps(data)
        self.pub_mqtt_msg.publish(mqtt_message)

    # MARK: Weight Balance
    def handle_weight_balance(self, data:str):
        json_data = json.loads(data)
        """
        json_data = {
            device_id: str
            mode: LATEST/STABLE
            job_id: Optional[str]
        }
        """
        command_lookup = {
            "LATEST" : WeightCommand.Request.MODE_LATEST,
            "STABLE" : WeightCommand.Request.MODE_STABLE,
        }
        request = WeightCommand.Request()
        device_id = json_data.get("device_id", "")
        weight_devices = self.external_devices[DeviceType.MICROBALANCE]
        if device_id not in weight_devices:
            self.publish_telemetry( socket_channel = "weight_balance_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Weight.ERROR,
                reason      = "Invalid device_id",
            )
            return
        request.job_id = json_data.get("job_id", "")
        request.mode = command_lookup.get(json_data["mode"], WeightCommand.Request.MODE_LATEST)
        service: Client = weight_devices[device_id].start_service
        if not service.service_is_ready():
            self.publish_telemetry( socket_channel = "weight_balance_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Weight.ERROR,
                reason      = "Service is not ready",
            )
            return
        future = service.call_async(request)
        self.publish_telemetry( socket_channel = "weight_balance_msg",
            device_id   = device_id,
            job_id      = request.job_id,
            success     = True,
            reason      = "Command send to service",
            status      = Weight.WORKING,
        )
    
    def send_weight_balance_status(self, device_id: str, data: Weight):
        # TODO: 
        data.stable
        data.tared
        # What to do with these two values
        self.publish_telemetry( socket_channel="weight_balance_msg",
            device_id   = device_id,
            job_id      = data.job_id,
            success     = True,
            status      = data.status,
            value       = {
                "value": round(data.value.value, 4),
                "unit" : data.value.unit,
            },
            reason      = data.reason,
        )
    # -----------Weight Balance Control: END-----------
        
    # MARK: RAVEN
    def handle_raven_msg(self, data:str):
        raven_msg = String(data=data)
        self.pub_raven_msg.publish(raven_msg)
    # -----------RAVEN Control: END-----------
    
    # MARK: Magenetic Stir
    def handle_magnetic_stir(self, data: str):
        self.logger.info(f"Got Magnetic Stirrer control command: {data}")
        json_data = json.loads(data)
        """
        json_data = {
            device_id: str
            rpm: int 
            temperature: Optional[int]
            timer: {
                value: float
                unit: str 
                }
            job_id: Optional[str]
        }
        """
        request = MagneticStirrerCommand.Request()
        device_id = json_data.get("device_id", "")
        magentic_stirrer_devices = self.external_devices[DeviceType.MAGNETIC_STIRRER]
        request.job_id = json_data.get("job_id", "")
        if not device_id in magentic_stirrer_devices:
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = "Invalid device_id",
            )
            return
        rpm = json_data.get("rpm", 0)
        if not isinstance(rpm, int):
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = f"`rpm`={rpm} must be a int",
            )
            return
        request.rpm = rpm
        temperature = json_data.get("temperature", 0.0)
        if not isinstance(temperature, float):
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = f"`temperature`={temperature} must be a float",
            )
            return
        request.temperature.value = float(temperature)
        timer = json_data.get("timer", None)
        if not timer:
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = f"Unable to find timer data",
            )
            return
        timer_value = timer.get("value", None)
        if not isinstance(timer_value, (int,float)):
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = f"`timer.value`={timer_value} must be a number",
            )
            return
        request.timer.value = float(timer_value)
        timer_unit = timer.get("unit", "")
        if not isinstance(timer_unit, str):
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = f"`timer.unit`={timer_unit} must be a string",
            )
            return
        request.timer.unit = timer_unit
        service: Client = magentic_stirrer_devices[device_id].start_service
        if not service.service_is_ready():
            self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = MagneticStirrer.ERROR,
                reason      = "Service is not ready",
            )
            return
        future = service.call_async(request)
        self.publish_telemetry( socket_channel = "magnetic_stirrer_msg",
            device_id   = device_id,
            job_id      = request.job_id,
            success     = True,
            reason      = "Command send to service",
            status      = MagneticStirrer.WORKING,
        )
    
    def send_magnetic_stirrer_status(self, device_id: str, data: MagneticStirrer):
        self.publish_telemetry( socket_channel="magnetic_stirrer_msg",
            device_id   = device_id,
            job_id      = data.job_id,
            success     = True,
            status      = data.status,
            #TODO: Alight the data type to the backend
            timer       = {
                "value": round(data.timer.value, 4),
                "unit" : data.timer.unit,
            },
            temperature       = {
                "value": round(data.temperature.value, 4),
                "unit" : data.timer.unit,
            },
            reason      = data.reason,
        )
    # -----------Magenetic Stir Control: END-----------


    # -----------Sonicator Control: START-----------
    # MARK:- Sonicator
    def handle_sonicator(self, data:str):
        self.logger.info(f"Got sonicator control command: {data}")
        json_data = json.loads(data)
        """
        json_data = {
            device_id: str
            type: HEAT/SONICATE
            temperature: Optional[{
                value: float
                unit: str
                }]
            timer:  {
                value: float
                unit: str 
                }
            job_id: Optional[str]
        }
        """
        request = SonicatorCommand.Request()
        device_id = json_data.get("device_id", "")
        sonicator_devices = self.external_devices[DeviceType.SONICATOR]
        request.job_id = json_data.get("job_id", "")
        if not device_id in sonicator_devices:
            self.publish_telemetry( socket_channel = "sonicator_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Sonicator.ERROR,
                reason      = "Invalid device_id",
            )
            return
        temperature = json_data.get("temperature", None)
        if temperature:
            try:
                request.temperature.value = temperature.get("value")
                request.temperature.unit  = temperature.get("unit")
            except :
                self.publish_telemetry( socket_channel = "sonicator_msg",
                    device_id   = device_id,
                    job_id      = request.job_id,
                    success     = False,
                    status      = Sonicator.ERROR,
                    reason      = "Error while parsing temperature",
                )
                return
        timer = json_data.get("timer", None)
        if not timer:
            self.publish_telemetry( socket_channel = "sonicator_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Sonicator.ERROR,
                reason      = f"Invalid `timer` data",
            )
            return
        try:
            request.timer.value = timer.get("value")
            request.timer.unit  = timer.get("unit")
        except :
            self.publish_telemetry( socket_channel = "sonicator_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Sonicator.ERROR,
                reason      = "Error while parsing `timer`",
            )
            return
        control_mode = json_data.get("control_mode", "")
        if control_mode == SonicatorCommand.Request.CONTROL_MODE_SONICATE:
            service: Client = sonicator_devices[device_id].start_service
        elif control_mode == SonicatorCommand.Request.CONTROL_MODE_HEAT:
            service: Client = sonicator_devices[device_id].start_heater
        else:
            self.publish_telemetry( socket_channel = "sonicator_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Sonicator.ERROR,
                reason      = f"Invalid Control Mode",
            )
            return
        if not service.service_is_ready():
            self.publish_telemetry( socket_channel = "sonicator_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = Sonicator.ERROR,
                reason      = "Service is not ready",
            )
            return
        future = service.call_async(request)
        self.publish_telemetry( socket_channel = "sonicator_msg",
            device_id   = device_id,
            job_id      = request.job_id,
            success     = True,
            reason      = "Command send to service",
            status      = Sonicator.WORKING,
        )

    def send_sonicator_status(self, device_id: str, data: Sonicator):
        self.publish_telemetry( socket_channel="sonicator_msg",
            device_id   = device_id,
            job_id      = data.job_id,
            success     = True,
            status      = data.status,
            timer       = {
                "value": round(data.timer.value, 4),
                "unit" : data.timer.unit,
            },
            temperature       = {
                "value": round(data.temperature.value, 4),
                "unit" : data.timer.unit,
            },
            reason      = data.reason,
        )
    # -----------Sonicator Control: END-----------

    # MARK: Centrifuge
    def handle_centrifuge(self, data: str):
        self.logger.info(f"Got Centrifuge control command: {data}")
        json_data = json.loads(data)
        """
        json_data = {
            device_id: str
            rpm: int 
            timer:  {
                value: float
                unit: str 
                }
            job_id: Optional[str]
        }
        """
        request = CentrifugeCommand.Request()
        device_id = json_data.get("device_id", "")
        centrifuge_devices = self.external_devices[DeviceType.CENTRIFUGE]
        request.job_id = json_data.get("job_id", "")
        if not device_id in centrifuge_devices:
            self.publish_telemetry( socket_channel = "centrifuge_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = CentrifugeStatus.ERROR,
                reason      = "Invalid device_id",
            )
            return
        rpm = json_data.get("rpm", None)
        if not isinstance(rpm, int):
            self.publish_telemetry( socket_channel = "centrifuge_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = CentrifugeStatus.ERROR,
                reason      = f"`rpm`={rpm} must be a int",
            )
            return
        request.rpm = rpm
        timer = json_data.get("timer", None)
        if not timer:
            self.publish_telemetry( socket_channel = "centrifuge_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = CentrifugeStatus.ERROR,
                reason      = f"Invalid `timer` data",
            )
            return
        timer_value = timer.get("value", None)
        if not isinstance(timer_value, (int, float)):
            self.publish_telemetry( socket_channel = "centrifuge_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = CentrifugeStatus.ERROR,
                reason      = f"`timer.value` must be number",
            )
            return
        request.timer.value = float(timer_value)
        timer_unit = timer.get("unit", None)
        if not timer_unit:
            self.publish_telemetry( socket_channel = "centrifuge_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = CentrifugeStatus.ERROR,
                reason      = f"`timer.unit` not found",
            )
            return
        request.timer.unit = timer_unit
        service: Client = centrifuge_devices[device_id].start_service
        if not service.service_is_ready():
            self.publish_telemetry( socket_channel = "centrifuge_msg",
                device_id   = device_id,
                job_id      = request.job_id,
                success     = False,
                status      = CentrifugeStatus.ERROR,
                reason      = "Service is not ready",
            )
            return
        future = service.call_async(request)
        self.publish_telemetry( socket_channel = "centrifuge_msg",
            device_id   = device_id,
            job_id      = request.job_id,
            success     = True,
            reason      = "Command send to service",
            status      = CentrifugeStatus.WORKING,
        )

    def send_centrifuge_status(self, device_id: str, data: CentrifugeStatus):
        self.publish_telemetry( socket_channel="centrifuge_msg",
            device_id   = device_id,
            job_id      = data.job_id,
            success     = True,
            status      = data.status,
            rpm         = data.rpm,
            timer       = {
                "value": round(data.timer.value, 4),
                "unit" : data.timer.unit,
            },
            reason      = data.reason,
        )

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationMaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
