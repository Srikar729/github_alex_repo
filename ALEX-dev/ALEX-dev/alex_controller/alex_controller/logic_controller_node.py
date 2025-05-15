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
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse 
# ROS Interface Import
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import String
from geometry_msgs.msg import Point
from alex_interfaces.msg import CobotUpdates, CobotAction, QuantityValue, ModeStatus
from alex_interfaces.srv import ( 
    PauseResumeAction, ArmCommand, MeasurePhCommand, LiquidDoserCommand, 
    WeightCommand, MagneticStirrerCommand, CentrifugeCommand, SonicatorCommand, ControlMode,
    MeasureWeights, HomogenizerCommand
)
from alex_interfaces.action import ExecuteActionFile
# Python Import
import asyncio
import threading
import time
# Utilities Import
from alex_utilities import async_retry_function, RetryException
from alex_utilities.common_utilities import change_case
from alex_lab_objects.alex_object import ALEXObject
from alex_lab_objects.alex_500_volumetric_flask import VolumetricFlask500
from alex_lab_objects.alex_200_bottle import Bottle200
from alex_lab_objects.alex_liquids import Liquid
from alex_lab_objects.alex_ph_meter import PHMeter
from alex_lab_objects.alex_homogenizer import Homogenizer
from alex_lab_objects import WeightBalance, WeightBoat, MagneticStirrer, Sonicator
from alex_lab_objects import VolumetricFlask200
from alex_lab_objects.alex_container import Container
from alex_lab_objects.alex_constans import ContainerType

from alex_utilities.device_configuration_handler import ExternalDeviceConfiguration, ConfigValidationError, DeviceType, RosDetailsType

class MixError(Exception):
    pass

class StirError(Exception):
    pass

class SonicationError(Exception):
    pass

class MeasureError(Exception):
    pass

class ModePermissionError(Exception):
    pass

class LogicControllerAction(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        # Default initializations
        self.logger = self.get_logger()
        self.parameters()
        # Service setups
        self.create_service_client()
        # Action setups
        self.action_initilize()
        self.set_subscriber()
        # For pause and resume service
        self.initilize()
        self.setup_object_trackers()
        self.logger.info(f"{node_name} started running")

    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('external_device_config_path', value='/home/dexus/ros2_ws/install/alex_bringup/share/alex_bringup/params/device_configuration.yaml',   descriptor=ParameterDescriptor(description="Path to the devices config file"))
        # get ros 2 parameters
        self.external_device_config_path:str   = self.get_parameter('external_device_config_path').value
    
    def create_service_client(self):
        self.alex_mode_regulator_srv = self.create_client(ControlMode, "alex_mode_regulator")

    def action_initilize(self):
        self._goal_handle:ServerGoalHandle  = None
        self._goal_lock                     = threading.Lock()
        # Initialize action server
        self._action_server = ActionServer(self,
            ExecuteActionFile,
            'execute_afi',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
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
            match (device_config.device_type):
                # MARK: __PH
                case DeviceType.PH_METER:
                    service_name  = device_ros_details.get_controller_service_name(device_name)
                    start_service = self.create_client(MeasurePhCommand, service_name)
                    device_ros_details.set_controller_service(start_service)
                # MARK: __Liquid Dispensor
                case DeviceType.LIQUID_DISPENSOR:
                    dosing_service_name = device_ros_details.get_dosing_service_name(device_name)
                    dosing_service      = self.create_client(LiquidDoserCommand, dosing_service_name)
                    device_ros_details.set_dosing_service(dosing_service)
                # MARK: __Microbalance
                case  DeviceType.MICROBALANCE:
                    service_name  = device_ros_details.get_controller_service_name(device_name)
                    start_service =  self.create_client(MeasureWeights, service_name)
                    device_ros_details.set_controller_service(start_service)
                # MARK: __Magnetic Stirrer
                case DeviceType.MAGNETIC_STIRRER:
                    service_name  = device_ros_details.get_service_name(device_name)
                    start_service = self.create_client(MagneticStirrerCommand, service_name)
                    device_ros_details.set_start_service(start_service)
                # MARK: __Homogenizer
                case DeviceType.HOMOGENIZER:
                    service_name  = device_ros_details.get_service_name(device_name)
                    start_service = self.create_client(HomogenizerCommand, service_name)
                    device_ros_details.set_start_service(start_service)
                # MARK: __Centrifuge
                case DeviceType.CENTRIFUGE:
                    service_name  = device_ros_details.get_service_name(device_name)
                    start_service = self.create_client(CentrifugeCommand, service_name)
                    device_ros_details.set_start_service(start_service)
                # MARK: __Sonicator
                case DeviceType.SONICATOR:
                    service_name  = device_ros_details.get_service_name(device_name)
                    start_service = self.create_client(SonicatorCommand, service_name)
                    device_ros_details.set_start_service(start_service)

                    heater_name   = device_ros_details.get_heater_name(device_name)
                    heater_service = self.create_client(SonicatorCommand, heater_name)
                    device_ros_details.set_start_heater(heater_service)
                # MARK: __Robot
                case DeviceType.ROBOT:
                    service_name  = device_ros_details.get_controller_service_name(device_name)
                    start_service = self.create_client(ArmCommand, service_name)
                    device_ros_details.set_controller_service(start_service)

            external_devices[device_config.device_type].update({ device_name: device_ros_details })
        return external_devices
    
    def get_robot_details(self):
        """Get the first robot details from the external devices"""
        if DeviceType.ROBOT not in self.external_devices:
            raise ValueError("Robot service not found in external devices")
        robot_devices = self.external_devices[DeviceType.ROBOT]
        if not robot_devices:
            raise ValueError("Robot service not found in external devices")
        robot_devices_values = robot_devices.values()
        robot_device = next(iter(robot_devices_values))
        return robot_device
    
    def set_subscriber(self):
        self.mode_regulator = self.create_subscription(ModeStatus, "mode_publisher", self.mode_publisher_sub, 2)
    
    def initilize(self):
        self._alex_controller_mode = ControlMode.Request.MODE_NONE
        self.action_execute_start_time = None
        self.total_steps    = 0
        self.current_index  = 0
        self._control_updates = PauseResumeAction.Request.RESUME
        self.manage_pause_resume = self.create_service(PauseResumeAction, "manage_pause_resume", self.handle_manage_pause_resume)
        self.get_current_time = lambda : round(time.time() * 1000)
        
        self.external_devices = self.load_external_devices(self.external_device_config_path)
        self.robot_service = self.get_robot_details()

    def mode_publisher_sub(self, data: ModeStatus):
        self._alex_controller_mode = data.current_mode

    def value_reset(self):
        """Reset the value to default once done with action"""
        self.total_steps    = 0
        self.current_index  = 0
        self.action_execute_start_time = None

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request:ServerGoalHandle):
        """Accept only one goal at a time. Reject any concurrent requests"""
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # Abort the new goal
                self.logger.warning("Got another goal while executing a goal")
                return GoalResponse.REJECT
        self.logger.info('Accepeted new goal')
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle:ServerGoalHandle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.logger.info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()
    
    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.logger.info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def handle_manage_pause_resume(self, request:PauseResumeAction.Request, response:PauseResumeAction.Response):
        request_dict = {
            PauseResumeAction.Request.STOP    : "STOP",
            PauseResumeAction.Request.RESUME  : "RESUME",
            PauseResumeAction.Request.PAUSE   : "PAUSE",
            PauseResumeAction.Request.GETMODE : "GETMODE",
        }
        self.logger.info(f"Got `manage_pause_resume` mode:{request_dict[request.mode]}")
        # Respond with the current status
        if request.mode == PauseResumeAction.Request.GETMODE:
            response.success = False
            response.status  = f"Mode={request_dict[self._control_updates]}"
            return response
        # Guard Cases
        if not self._goal_handle or not self._goal_handle.is_active:
            response.success = False
            response.status  = "No active goal found"
            return response
        if self._control_updates == request.mode:
            response.success = False
            response.status  = "Already in the same mode"
            return response
        # Feedback to the action
        status = {
            PauseResumeAction.Request.STOP  : CobotUpdates.CANCELLING,
            PauseResumeAction.Request.RESUME: CobotUpdates.RESUMING,
            PauseResumeAction.Request.PAUSE : CobotUpdates.PAUSING,
        }.get(request.mode, None)
        if status is not None:
            feedback = ExecuteActionFile.Feedback()
            feedback.status.status = status
            feedback.status.total_steps = self.total_steps
            feedback.status.current_step_index = self.current_index
            feedback.status.start_timestamp = self.action_execute_start_time
            feedback.status.end_timestamp = self.get_current_time()
            feedback.status.job_id = self._goal_handle.request.job_info
            self._goal_handle.publish_feedback(feedback)
        # Assign the latest updates
        self._control_updates = request.mode
        response.success = True
        return response

    def setup_object_trackers(self):
        self.objects: dict[str, dict[str, ALEXObject]] = {
            'container': {
                'container-1': WeightBoat(                            Point(x=-476.00,    y=625.41, z=132.00), self.robot_service.controller_service, self.logger),
                'container-2': Container(ContainerType.ML_200_BEAKER, Point(x=-763.78,    y=238.54, z=147.00), self.robot_service.controller_service, self.logger),
                'container-3': VolumetricFlask500(                    Point(x=-501.950,   y=500.220,z=160.00), self.robot_service.controller_service, self.logger),
            },
            'location': {
                'location-1': ALEXObject(Point(x=-484.22,  y=-593.86, z=155.23)),
                'location-2': ALEXObject(Point(x=-493.96,  y=510.17,  z=232.44)),
            },
            "liquids": {
                # BUG: The external liquid ID should come from environment list from backend
                # BUG: The liquid-1 should be somehow linked to the device
                "liquid-1": Liquid(Point(x=-575.00, y=-165.00, z=122.00), self.external_devices[DeviceType.LIQUID_DISPENSOR]["liquid-dispenser-c9f82d40"].dosing_service, logger=self.logger, should_raise=True),
                "liquid-2": Liquid(Point(x=-585.00, y=-302.00, z=122.00), self.external_devices[DeviceType.LIQUID_DISPENSOR]["liquid-dispenser-48f6b503"].dosing_service, logger=self.logger, should_raise=True),
            }
        }
        self.devices_objects = {
            # BUG: The external devices should come from environment list from backend
            "pH"     : PHMeter      (Point(x=-42.00,   y=-613.00, z=187.00), self.external_devices[DeviceType.PH_METER]["pH-meter-d0378087"].controller_service),
            "weight" : WeightBalance(Point(x=-343.00,  y=-610.00, z=160.00), self.external_devices[DeviceType.MICROBALANCE]["weight-balance-795f325c"].controller_service, actuator_exist=False),
            # "magnetic-stirrer" : MagneticStirrer(Point(x=-484.22,  y=-593.86, z=146.00), self.external_devices[DeviceType.MAGNETIC_STIRRER]["magnetic-stirrer-667cdc5a"].start_service),
            # "sonicator" : Sonicator(Point(x=-93.21,  y=863.62, z=38.57), self.external_devices[DeviceType.SONICATOR]["sonicator-b337baa6"].start_service, self.external_devices[DeviceType.SONICATOR]["sonicator-b337baa6"].start_heater),
        }
    
    def get_object_reference(self, name: str) -> ALEXObject:
        for key in self.objects:
            for obj in self.objects[key]:
                if name == obj:
                    return self.objects[key][obj]
        raise ValueError('Invalid target object, not configured in local reference')

    def get_action_obj_refs(self, action_file_obj_ref, action: CobotAction) -> tuple[ALEXObject, ALEXObject]:
        target_obj = action.target
        param_obj = action.parameter
        target_obj = action_file_obj_ref[target_obj]
        target_reference = self.get_object_reference(target_obj)
        if action.action in ["measure"]:
            if action.parameter not in self.devices_objects: raise ValueError("Device to measure not configured in local reference")
            else: param_reference = self.devices_objects[action.parameter]
        elif action.action in ["stir"]:
            param_reference = int(param_obj)
        elif action.action in ["sonicate"]:
            param_reference = float(param_obj)
        else:
            param_obj = action_file_obj_ref[param_obj]
            param_reference = self.get_object_reference(param_obj)
        return target_reference, param_reference

    async def execute_move(self, target: Container, param: ALEXObject):
        await target.prep()
        await target.pick()
        await target.move(param.position)
        await target.place(param.position)

    async def execute_add(self, target: Container, param: ALEXObject, quantity: QuantityValue, job_id: str):
        target_position = target.position
        await target.prep()
        await target.pick()
        
        #TODO: Should convert to be more generic
        if type(param) == Container:
            position = Point(x=param.position.x, y=param.position.y, z=param.position.z) # had to create new point as param is pass by reference
            # pouring height
            position.z = param.size["height"] + 150.0
            # pouring side distance
            #TODO: quadrent specific, should fix it
            position.y -= (param.size["width"]/2) + (target.size["width"]/2) + 20.0

            # pour the liquid from the container
            await target.move(position, pre_destination=False) # Over calculation
            await target.pour()
            pass
        elif type(param) == Liquid:
            position = param.position
            await target.move(position, pre_destination=False)
            # the displace liquid
            #BUG: Do not directly send value. Should do type conversion if required by the service
            value = quantity.value
            await async_retry_function(param.pour, value, job_id, logger=self.logger)

        await target.move(target_position)
        await target.place(target_position)

    async def execute_stir(self, target: Container, param: int, quantity: QuantityValue, job_id: str):
        target_position = target.position
        await target.prep()
        await target.pick()
        
        if 0 <= param < 10:
            await target.stir()
        elif param < 100:
            StirError(f"Invalid stir parameter: Can't stir at speed={param}")
        else:
            # Move to the stirrer
            # TODO: Should find the available stirrer
            stirrer:MagneticStirrer = self.devices_objects["magnetic-stirrer"]
            # # ------------------
            # # IN PROGRESS
            # # ------------------
            # stirrer_position = stirrer.position
            # await target.move(stirrer_position)
            # # Place the container
            # await target.place(stirrer_position)
            # # ------------------
            # Do stir
            await async_retry_function(stirrer.stir, param, quantity, job_id, logger=self.logger)
            # # ------------------
            # # IN PROGRESS
            # # ------------------
            # # Pick container
            # await target.pick()
            # # Move back
            # await target.move(target_position)
            # # ------------------

        await target.place(target_position)
    
    async def execute_measure(self, target: Container, param: PHMeter | WeightBalance, job_id: str = ""):
        target_position = target.position

        if isinstance(param, WeightBalance):
            try: await param.open_door()
            except Exception as e: raise MeasureError(e)
        await target.prep()
        await target.pick()
        await target.move(param.position)
        await target.place(param.position)
        
        try: measured_value =  await async_retry_function(param.measure, target, job_id, logger=self.logger)
        except RetryException as e:
            raise
        except Exception as e:
            raise MeasureError(e)
        self.logger.info(f"Got measured value as: {measured_value}")
        
        await target.pick()
        await target.move(target_position)
        await target.place(target_position)
        if isinstance(param, WeightBalance):
            try: await param.open_door()
            except Exception as e: raise MeasureError(e)
        return measured_value
    
    async def execute_sonicate(self, target: Container, param: int, quantity: QuantityValue, job_id: str):
        target_position = target.position

        if target.type not in [ContainerType.ML_200_VOLUMETRIC_FLASK, ContainerType.ML_500_VOLUMETRIC_FLASK]:
            raise SonicationError("Unable to sonicate this container yet")
        
        sonicator:Sonicator = self.devices_objects["sonicator"]
        await target.prep()
        await target.pick()
        # TODO: Need to add retry once the sonicator is fully ready for testing
        try: 
            await sonicator.heating(quantity, job_id)
        except Exception as e:
            raise SonicationError(f"Heating error: {e}")
        try: 
            await sonicator.sonicate(float(param), quantity, job_id)
        except Exception as e:
            raise SonicationError(f"Sonication error: {e}")
        await target.place(target_position)


    async def alex_controller_mode(self, setter: bool = True):
        if not self.alex_mode_regulator_srv.service_is_ready():
            raise ModePermissionError("Service is not ready, to set alex mode")
        mode_request = ControlMode.Request()
        mode_request.origin = ControlMode.Request.ORIGIN_AFI
        if setter:
            mode_request.mode = ControlMode.Request.MODE_AFI
        else:
            mode_request.mode = ControlMode.Request.MODE_NONE
        mode_response = self.alex_mode_regulator_srv.call_async(mode_request)
        await mode_response
        mode_result:ControlMode.Response = mode_response.result()
        if not mode_result.success:
            raise ModePermissionError(f"Failed to switch to AFI mode, reason: {mode_result.reason}")
        self._alex_controller_mode = mode_result.current_mode
        self.logger.info("Got alex controller mode permission to execute action")
    
    def control_updates_checker(self, goal_handle:ServerGoalHandle, feedback:ExecuteActionFile.Feedback):
        # Pause action till you get resume
        send_pause_feedback = True
        while self._control_updates == PauseResumeAction.Request.PAUSE:
            self.logger.info(f"Paused!!! Waiting for 2 seconds")
            feedback.status.status = CobotUpdates.PAUSED
            if send_pause_feedback:
                feedback.status.current_step_index = self.current_index
                feedback.status.end_timestamp = self.get_current_time()
                goal_handle.publish_feedback(feedback)
                send_pause_feedback = False
                self.action_execute_start_time = self.get_current_time()
            time.sleep(2)
        # External cancel
        if self._control_updates == PauseResumeAction.Request.STOP:
            self.logger.warning(f"Got stop action")
            goal_handle.abort()
    
    async def execute_callback(self, goal_handle:ServerGoalHandle):
        # MARK: Handle action request
        try: # TODO: Could use decorator for cleaner code.(Optional)
            # Set the controls to resume/continue in the beginning
            self._control_updates = PauseResumeAction.Request.RESUME
            goal_request: ExecuteActionFile.Goal = goal_handle.request
            job_info = goal_request.job_info
            object_references = {item.reference: item.object for item in goal_request.afi.object_references}
            self.total_steps = len(goal_request.afi.actions)
            # MARK: Alex Mode Permission
            if self._alex_controller_mode != ControlMode.Request.MODE_AFI:
                await self.alex_controller_mode()
            # Sending ACK back to server
            feedback = ExecuteActionFile.Feedback()
            feedback.status.status = CobotUpdates.WORKING
            feedback.status.total_steps = self.total_steps
            feedback.status.current_step_index = 0
            feedback.status.start_timestamp = self.get_current_time()
            feedback.status.end_timestamp = 0
            feedback.status.job_id = job_info
            goal_handle.publish_feedback(feedback)
            # Start executing
            for self.current_index, action in enumerate(goal_request.afi.actions, start=1):
                self.action_execute_start_time = self.get_current_time()
                feedback.status.start_timestamp = self.action_execute_start_time
                if self.current_index < goal_request.start_index: 
                    continue
                target, param = self.get_action_obj_refs(object_references, action)
                self.logger.info(f"Running step {self.current_index}, action: {action.action}")
                # Handle each action type
                if action.action == 'move':
                    await self.execute_move(target, param)
                elif action.action == 'add':
                    await self.execute_add(target, param, quantity=action.quantity, job_id=job_info) 
                elif action.action == 'stir':
                    await self.execute_stir(target, param, action.quantity, job_id=job_info) 
                elif action.action == 'measure':
                    measured_value = await self.execute_measure(target, param, job_info)
                elif action.action == 'sonicate':
                    await self.execute_sonicate(target, param, action.quantity, job_info)
                else:
                    result = ExecuteActionFile.Result()
                    result.success = False
                    result.reason  = f'Unsupported action type: {action.action}'
                    result.status.status = CobotUpdates.ERROR
                    result.status.total_steps = self.total_steps
                    result.status.current_step_index = self.current_index
                    result.status.start_timestamp = 0
                    result.status.end_timestamp = self.get_current_time()
                    result.status.job_id = job_info
                    return result
                self.control_updates_checker(goal_handle, feedback)
                if goal_handle.is_cancel_requested or not goal_handle.is_active:
                    self.logger.warning("Got cancel request, I am dead.")
                    # Completed response 
                    result = ExecuteActionFile.Result()
                    result.success = True
                    result.reason  = "Cancelled by the user"
                    result.status.status = CobotUpdates.CANCELLED
                    result.status.total_steps = self.total_steps
                    result.status.current_step_index = self.current_index
                    result.status.start_timestamp = 0
                    result.status.end_timestamp = self.get_current_time()
                    result.status.job_id = job_info
                    return result
                feedback = ExecuteActionFile.Feedback()
                feedback.status.status = CobotUpdates.COMPLETED
                feedback.status.total_steps = self.total_steps
                feedback.status.current_step_index = self.current_index
                feedback.status.start_timestamp = self.action_execute_start_time
                feedback.status.end_timestamp = self.get_current_time()
                feedback.status.job_id = job_info
                goal_handle.publish_feedback(feedback)
                # Loop end
            self.logger.info("Action completed!!")
            goal_handle.succeed()
            # Completed response 
            result = ExecuteActionFile.Result()
            result.success = True
            result.status.status = CobotUpdates.COMPLETED
            result.status.total_steps = self.total_steps
            result.status.current_step_index = self.total_steps
            result.status.end_timestamp = self.get_current_time()
            result.status.job_id = job_info
            return result
        except Exception as e:
            # MARK: Handle Exception
            # Return Goal State
            result = ExecuteActionFile.Result()
            result.success = False
            result.reason = repr(e)
            result.status.status = CobotUpdates.ERROR
            result.status.total_steps = self.total_steps
            result.status.current_step_index = self.current_index
            result.status.start_timestamp = 0
            result.status.end_timestamp = self.get_current_time()
            result.status.job_id = job_info
            return result
        finally:
            # Clear of the value
            self.value_reset()
            try:
                await self.alex_controller_mode(setter=False)
            except Exception as e:
                self.logger.error("Failed to give back the control mode.. Check it out.")
        

def main(args=None):
    rclpy.init(args=args)
    action_server = LogicControllerAction()
    # Run the executor within the asyncio event loop
    loop = asyncio.get_event_loop()
    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    loop.run_until_complete(rclpy.spin(action_server, executor=executor))
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Test terminal Action Command
"""
ros2 action send_goal /execute_afi alex_interfaces/action/ExecuteActionFile '{ afi: {
version: 0.0,
author: "",
automation_target: "",
object_references: [
{ reference: "Bottle 250ml 1", object: "container-0" },
{ reference: "Beaker 250ml 1", object: "container-2" },
{ reference: "Volumetric Flask 250ml 1", object: "container-3" },
{ reference: "Location 1", object: "location-0" },
{ reference: "Location 2", object: "location-1" },
{ reference: "Location 3", object: "location-2" },
{ reference: "Location 4", object: "location-3" },
{ reference: "Active Bottle Point", object: "location-4" },
{ reference: "Active Flask Point", object: "location-5" },
{ reference: "Storage Bottle Point", object: "location-6" },
{ reference: "Storage Flask Point", object: "location-7" } 
],
actions: [
{
    "action": "move",
    "target": "Bottle 250ml 1",
    "parameter": "Active Bottle Point"
},
{
    "action": "move",
    "target": "Volumetric Flask 250ml 1",
    "parameter": "Active Flask Point"
},
{
    "action": "stir",
    "target": "Volumetric Flask 250ml 1",
    "parameter": "Active Bottle Point"
},
{
    "action": "add",
    "target": "Volumetric Flask 250ml 1",
    "parameter": "Beaker 250ml 1"
},
{
    "action": "add",
    "target": "Bottle 250ml 1",
    "parameter": "Beaker 250ml 1"
},
{
    "action": "move",
    "target": "Bottle 250ml 1",
    "parameter": "Storage Bottle Point"
},
{
    "action": "move",
    "target": "Volumetric Flask 250ml 1",
    "parameter": "Storage Flask Point"
}
]},
start_index: 0 }'
"""

"""
ros2 action send_goal /execute_afi alex_interfaces/action/ExecuteActionFile '{ afi: {
version: 0.0,
author: "",
automation_target: "",
object_references: [
{ reference: "Beaker 250ml 1", object: "container-2" },
{ reference: "Weight Boat", object: "container-1" },
{ reference: "H2O", object: "liquid-1" },
],
actions: [
{
    "action": "measure",
    "target": "Weight Boat",
    "parameter": weight
}
]},
start_index: 0 }'
"""

# MARK: Sonicator
"""
ros2 action send_goal /execute_afi alex_interfaces/action/ExecuteActionFile '{ afi: {
version: 0.0,
author: "",
automation_target: "",
object_references: [
{ reference: "Volumetric 250ml 1", object: "container-3" },
],
actions: [
{
    "action": "sonicate",
    "target": "Volumetric 250ml 1",
    "parameter": 1,
    "quantity": {
            "value": 25.0,
            "unit" : "C",
        }
}
]},
start_index: 0 }'
"""
