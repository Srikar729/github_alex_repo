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
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.srv import LiquidDoserCommand
from alex_interfaces.msg import SerialMessage, LiquidDosingStatus
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.external_constants import SerialBand
# Python Import
from time import sleep
from collections import deque

class LiquidDoser(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.parameters()
        self.initialize()
        self.set_publishers()
        self.set_subscriber()
        self.set_services()
        self.service_client()
        self.configure_device()
        self.logger.info(f"{node_name} started running")

    def parameters(self):
        # declare ros 2 parameters
        self.declare_parameter('max_acknowledgement_duration', 10)
        self.declare_parameter('max_completion_duration', 60)
        # get ros 2 parameters
        self.max_acknowledgement_duration:int= self.get_parameter('max_acknowledgement_duration').value
        self.max_completion_duration:int     = self.get_parameter('max_completion_duration').value

        self.max_acknowledgement_duration:Duration = Duration(seconds=self.max_acknowledgement_duration)
        self.max_completion_duration:Duration      = Duration(seconds=self.max_completion_duration)
    
    def set_publishers(self,):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_liquid_doser_status = self.create_publisher(LiquidDosingStatus, "liquid_dosing_status", 5)

    def set_subscriber(self,):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())
    
    def set_services(self,):
        self.create_service(LiquidDoserCommand, "start_liquid_dosing", self.handle_liquid_dosing, callback_group=ReentrantCallbackGroup() )

    def initialize(self,):
        self.serial_data_deque:deque[str] = deque(maxlen=2)
        command_keys = [item for item in dir(LiquidDoserCommand.Request) if item.startswith("COMMAND_")]
        self.command_dict = { getattr(LiquidDoserCommand.Request, command): command  for command in command_keys }
    
    # ---- Serial Device setup -----
    def service_client(self):
        serial_node_name = self.get_namespace() + "/serial_interface_lifecycle/change_state"
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
    # ---- Serial Device setup:END -----
    
    @property
    def serial_data(self):
        return self.serial_data_deque and self.serial_data_deque.popleft() or ""

    def wait_for_acknowledgment(self, request:LiquidDoserCommand.Request) -> tuple[bool, str]:
        """Waits for serial data and returns success status and reason"""
        if request.command == LiquidDoserCommand.Request.COMMAND_STOP:
            self.logger.info("Not waiting for ACk in stop command")
            return True, "Not waiting for serial"
        
        initial_time = self.get_clock().now()
        previous_runtime = 0
        while True:
            runtime = self.get_clock().now() - initial_time
            if runtime.to_msg().sec != previous_runtime:
                self.logger.info(f"Acknowledgment Run time: {runtime.to_msg().sec }")
                previous_runtime = runtime.to_msg().sec
            
            serial_data = self.serial_data
            
            if runtime > self.max_acknowledgement_duration:
                return False, "Timed out waiting for acknowledgment"
            elif not serial_data:
                # Wait to get some data
                pass
            elif "*OK" in serial_data:
                return True, "Got Acknowledgment"
            elif "*ER" in serial_data:
                return False, "Dispensor returned `unknown command`"
            elif "*OV" in serial_data: 
                return False, "over volt (VCC>=5.5V)"
            elif "*UV" in serial_data: 
                return False, "under volt (VCC<=3.1V)"
            elif "*RS" in serial_data: 
                return False, "reset"
            elif "*RE" in serial_data: 
                return False, "boot up complete, ready"
            elif "*SL" in serial_data: 
                return False, "entering sleep mode"
            elif "*WA" in serial_data: 
                return False, "wake up"
            elif "*MINVOL" in serial_data: 
                return False, "dispense amount too low"
            elif "*TOOFAST" in serial_data: 
                return False, "ml/min set to fast"
            sleep(0.5)
    
    def wait_for_response(self, ):
        initial_time = self.get_clock().now()
        previous_runtime = 0
        is_acknowledgment = lambda : serial_data.startswith("*OK")

        while True:
            runtime = self.get_clock().now() - initial_time
            if runtime.to_msg().sec != previous_runtime:
                self.logger.info(f"Response Run time: {runtime.to_msg().sec }")
                previous_runtime = runtime.to_msg().sec

            serial_data = self.serial_data

            if runtime > self.max_completion_duration:
                return False, "Timed out: from motor"
            if not is_acknowledgment() and (serial_data.startswith("?") or serial_data.startswith("*")):
                return True, serial_data
            sleep(0.5)
        return 

    def response_parse_done(self, response:str):
        response = response.replace("*DONE,", "")
        if not response:
            return None
        
        value = None
        try:
            value = float(response)
        except ValueError as e:
            self.logger.error(f"Unable to parse done. Error: {e}")
        self.logger.info(f"Dispensed value: {value}")
        return value
            
    def ros_publisher_decorator(func):
        def wrapper(self, request:LiquidDoserCommand.Request , response:LiquidDoserCommand.Response):
            result:LiquidDoserCommand.Response = func(self, request, response)
            result.status.job_id = request.job_id
            self.pub_liquid_doser_status.publish(result.status)
            return result
        return wrapper
    
    @ros_publisher_decorator
    def handle_liquid_dosing(self, request:LiquidDoserCommand.Request, response:LiquidDoserCommand.Response):
        """Atlas guideline.
        | Command | Value | Optional | Response             |
        | :---    | :---  | :-----:  | :------------------: |
        | D       | ml    | min      | *OK                  |
        | D       |  -    | ?        | ?D,ml,state & *OK    |
        | DC      | ml    | min      | *OK                  |
        | P       |  -    | ?        | ?P,state    & *OK    |
        | X       |  -    |  -       | *DONE,ml             |
        | Invert  |  -    |  -       | *OK                  |
        | Invert  |  -    | ?        | ?Invert,status & *OK |
        | TV      |  -    | ?        | ?TV,ml               |
        | ATV     |  -    | ?        | ?ATV,ml              |
        | Cal     | ml    |  -       | *OK                  |
        | Cal     |  -    | clear    | *OK                  |
        | Cal     |  -    | ?        | ?Cal,state   & *OK   |
        | PV      |  -    | ?        | ?PV,value    & *OK   |
        | i       |  -    |  -       | ?i,PMP,version       |
        | Status  |  -    |  -       |                      |
        | Sleep   |  -    |  -       |                      |
        """
        self.logger.info(f"Got requst command={self.command_dict.get(request.command, None)}, value={request.value}, optional={request.optional}")
        response.status.value.unit = "ml"

        # TODO: Currently does not handle parallel commands. Need to for handling Pause, Stop.

        # Parse and serial request
        command_str = request.command

        if request.value:
            command_str += f",{round(request.value, 3)}"

        if request.optional:
            command_str += f",{request.optional}"
        
        self.serial_data_deque.clear()
        self.serial_in.publish(SerialMessage(data=command_str))

        if request.optional not in [LiquidDoserCommand.Request.OPTIONAL_CHECK, LiquidDoserCommand.Request.OPTIONAL_CLEAR]:
            success, reason = self.wait_for_acknowledgment(request)
            self.logger.info(f"Acknowledgment response {success=}, {reason=}")
            if not success:
                self.logger.warning(f"Waiting for acknowledgment failed with {reason=}")
                response.success = success
                response.status.status = LiquidDosingStatus.ERROR
                response.status.reason = reason
                return response
            elif request.command == LiquidDoserCommand.Request.COMMAND_CALIBRATE:
                self.logger.info(f"Calibration completed")
                response.success = success
                response.status.status = LiquidDosingStatus.COMPLETED
                response.status.reason = "calibration_success"
                return response

        success, response_str = self.wait_for_response()
        if not success:
            self.logger.warning(f"Waiting for respose failed with {response_str=}")
            response.success = success
            response.status.status = LiquidDosingStatus.ERROR
            response.status.reason = response_str
            return response
        self.logger.info(f"Got Response {success=}, {response_str=}")
        
        # Parse response
        if "*DONE" in response_str:
            # *DONE,ml
            value = self.response_parse_done(response_str)
            response.status.value.value = value or 0.0            
            response.status.status = LiquidDosingStatus.COMPLETED if request.command != LiquidDoserCommand.Request.COMMAND_STOP else LiquidDosingStatus.CANCELLED

            response.status.status = (
                LiquidDosingStatus.COMPLETED, # FALSE = 0
                LiquidDosingStatus.CANCELLED, # TRUE  = 1
            )[request.command == LiquidDoserCommand.Request.COMMAND_STOP]

            response.success = True
            return response
        
        response.success = True
        response.status.status = LiquidDosingStatus.WORKING
        response.status.reason = response_str
        return response
        
    def handle_serial_out(self, data:SerialMessage):
        if not data.data.startswith(SerialBand.LIQUID_DOSING.value): return
        value = data.data.removeprefix(SerialBand.LIQUID_DOSING.value)
        self.logger.debug(f"{value}")
        self.serial_data_deque.append(value)
    
def main(args=None):
    rclpy.init(args=args)
    node = LiquidDoser()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
