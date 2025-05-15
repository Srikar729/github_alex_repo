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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.msg import SerialMessage, QuantityValue
from alex_interfaces.srv import StepperActuatorCommand
# Python Import
from enum import Enum
from time import sleep
from queue import Queue, Empty as QueueEmpty
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.external_constants import SerialBand

class MotorStatus(Enum):
    ACKNOWLEDGEMENT = "ACK:"
    LIMIT           = "LIMIT:"
    DONE            = "DONE:"

class StepperActuator(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        
        self.parameters()
        self.initialize()
        self.set_publishers()
        self.set_subscribers()
        self.set_services()

        self.service_client()
        self.configure_device()
        
        self.logger.info(f"{node_name} started running")
    
    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('max_stroke_length', 335)
        self.declare_parameter('max_acknowledgement_duration', 10)
        self.declare_parameter('max_completion_duration', 60)
        # get ros 2 parameters
        self.max_stroke_length:float                 = self.get_parameter('max_stroke_length').value
        self.max_acknowledgement_duration_seconds:int= self.get_parameter('max_acknowledgement_duration').value
        self.max_completion_duration_seconds:int     = self.get_parameter('max_completion_duration').value

    def initialize(self):
        self.device_connected = False
        self.serial_data: Queue[str] = Queue()
        self.current_position = QuantityValue(value=-1.0, unit="mm")
    
    def set_publishers(self,):
        self.pub_serial_in  = self.create_publisher(SerialMessage, 'to_serial', 10)

    def set_subscribers(self,):
        self.sub_serial_out = self.create_subscription(SerialMessage, 'from_serial', self.handle_serial_data, 10, callback_group=ReentrantCallbackGroup())

    def set_services(self,):
        self.srv = self.create_service(StepperActuatorCommand, 'move_actuator', self.control_actuator, callback_group=ReentrantCallbackGroup())

    # MARK: Serial Device Setup 
    def service_client(self):
        namespace = "" if self.get_namespace() == "/" else self.get_namespace()
        serial_node_name = namespace + "/serial_interface_lifecycle/change_state"
        self.change_state_srv = self.create_client(ChangeState, serial_node_name)

    def configure_device(self,):
        self.logger.info("Configuring Device")
        wait_time = 20
        for i in range(wait_time):
            if self.change_state_srv.wait_for_service(1):
                break
            self.logger.info(f"Serial lifecycle service waiting. Waiting: {wait_time - i}")
        else:
            self.logger.error("Could not find the device serial node, try again..")
            return
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.change_state_srv.call_async(change_state_req)
        future.add_done_callback(self.configure_callback)
    
    def configure_callback(self, future: Future):
        result: ChangeState.Response = future.result()
        if not result.success:
            self.logger.error("Failed to activate the device. Will try again in 2 seconds")
            sleep(2)
            self.configure_device()
            return
        self.logger.info("Configuring Device: Success")
        self.activate_device()
    
    def activate_device(self,):
        self.logger.info("Activating Device")
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.change_state_srv.call_async(change_state_req)
        future.add_done_callback(self.activate_callback)

    def activate_callback(self, future: Future):
        result: ChangeState.Response = future.result()
        if not result.success:
            self.logger.error("Failed to activate the device. Will try again in 2 seconds")
            sleep(2)
            self.activate_device()
            return
        self.logger.info("Activating Device: Success")
        # MARK:__Initial Homing
        request = StepperActuatorCommand.Request()
        request.distance.value = self.max_stroke_length + 50.0 # Going above max limit
        request.distance.unit  = "mm"
        self.control_actuator(request, StepperActuatorCommand.Response())

    # ---- Serial Device setup:END -----

    # MARK: Control Logic
    def control_actuator(self, request: StepperActuatorCommand.Request, response: StepperActuatorCommand.Response):
        self.logger.info(f"Got Request direction: {request.distance.value} {request.distance.unit}")

        if not self.device_connected:
            response.current_position = self.current_position
            response.status = StepperActuatorCommand.Response.STATUS_ERROR
            response.reason = "Actuator device is not connected"
            return response
        
        available_distance_units = {
            StepperActuatorCommand.Request.UNIT_MILLIMETER : 1,
            StepperActuatorCommand.Request.UNIT_CENTIMETER : 10,
            StepperActuatorCommand.Request.UNIT_METER      : 1000,
        }

        distance_mm = request.distance.value * available_distance_units.get(request.distance.unit, 0)

        if distance_mm == 0 != request.distance.value:
            response.status = StepperActuatorCommand.Response.STATUS_ERROR
            response.current_position = self.current_position
            response.reason = f"Got invalid unit, available units are {','.join(available_distance_units.keys())}"
            self.logger.error(f"{response.reason}")
            return response 

        serial_command = SerialMessage(data=f"{distance_mm}")
        
        self.pub_serial_in.publish(serial_command)
        try: 
            serial_response = self.serial_data.get(timeout=self.max_acknowledgement_duration_seconds)
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.max_acknowledgement_duration_seconds} s")
            response.current_position = self.current_position
            response.status = StepperActuatorCommand.Response.STATUS_ERROR 
            response.reason = "TIMEOUT: Failed to get serial response"
            return response
        
        if serial_response.startswith(MotorStatus.ACKNOWLEDGEMENT.value):
            self.logger.info(f"Got acknowledgment. Executing motion: {serial_response}")

        try: 
            serial_response = self.serial_data.get(timeout=self.max_completion_duration_seconds)
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.max_completion_duration_seconds} s")
            response.current_position = self.current_position
            response.status = StepperActuatorCommand.Response.STATUS_ERROR
            response.reason = "TIMEOUT: Failed to get serial response"
            return response
        
        if serial_response.startswith(MotorStatus.ACKNOWLEDGEMENT.value):
            response.status = StepperActuatorCommand.Response.STATUS_ERROR
            response.current_position = self.current_position
            response.reason = "Got some new command, Aborting current"
            return response
        
        if serial_response.startswith(MotorStatus.LIMIT.value):
            direction = serial_response.removeprefix(MotorStatus.LIMIT.value)
            match direction:
                case StepperActuatorCommand.Request.DIRECTION_POSITIVE:
                    self.current_position.value = float(self.max_stroke_length)
                case StepperActuatorCommand.Request.DIRECTION_NEGATIVE:
                    self.current_position.value = 0.0
            response.current_position = self.current_position
            response.status = StepperActuatorCommand.Response.STATUS_COMPLETED
            response.reason = f"Reached maximum {direction} limit"
            self.logger.warning(response.reason)
            return response
        
        if serial_response.startswith(SerialBand.ERROR.value):
            parsed_data = serial_response.removeprefix(SerialBand.ERROR.value)
            response.current_position = self.current_position
            response.status = StepperActuatorCommand.Response.STATUS_ERROR
            response.reason = parsed_data

        if serial_response == MotorStatus.DONE.value:
            self.current_position.value = distance_mm
            response.current_position = self.current_position
            response.status = StepperActuatorCommand.Response.STATUS_COMPLETED
            return response

        response.current_position = self.current_position
        response.status = StepperActuatorCommand.Response.STATUS_COMPLETED
        response.reason = f"Got unexpected data: {serial_response}"
        self.logger.info(response.reason)
        return response
    
    def handle_serial_data(self,  serial_data:SerialMessage):
        self.device_connected = bool(serial_data.state)
        if not serial_data.data.startswith(SerialBand.STEPPER_ACTUATOR.value):
            self.logger.warning(f"Got invalid: {serial_data.data}")
            return
        self.serial_data.put(serial_data.data.removeprefix(SerialBand.STEPPER_ACTUATOR.value))

def main(args=None):
    rclpy.init(args=args)
    node = StepperActuator()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
