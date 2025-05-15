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
from rclpy.logging import LoggingSeverity
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.srv import DCActuator as DCActuatorSrv
from alex_interfaces.msg import SerialMessage
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.external_constants import SerialBand
# Python Import
from time import sleep
from queue import Queue, Empty as QueueEmpty

class DCActuator(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.get_logger().set_level(LoggingSeverity.DEBUG)        

        self.initialize()
        self.set_publisher()
        self.set_subscriber()
        self.set_service()
        self.service_client()
        self.configure_device()

    def initialize(self):
        self.rate = self.create_rate(0.5)
        self.serial_data: Queue[str] = Queue()
        self.device_connected = False
        self.serial_timeout = 10
        self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
        command_keys = [item for item in dir(DCActuatorSrv.Request) if item.startswith("DIRECTION_")]
        self.command_dict = { getattr(DCActuatorSrv.Request, command): command  for command in command_keys }

    def set_publisher(self):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
    
    def set_subscriber(self):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())

    def set_service(self):
        self.srv = self.create_service(DCActuatorSrv, 'move_actuator', self.control_actuator, callback_group=ReentrantCallbackGroup())

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
    # ---- Serial Device setup:END -----
    
    # MARK: Control Logic
    def control_actuator(self, request: DCActuatorSrv.Request, response: DCActuatorSrv.Response):
        self.logger.info(f"Got Request direction: {self.command_dict.get(request.direction)}")

        direction_serial_lookup = {
            DCActuatorSrv.Request.DIRECTION_FORWARD : "F",
            DCActuatorSrv.Request.DIRECTION_STOP    : "S",
            DCActuatorSrv.Request.DIRECTION_BACKWARD: "B",
        }

        serial_command = direction_serial_lookup.get(request.direction)
        if not serial_command:
            response.status = DCActuatorSrv.Response.STATUS_ERROR
            response.reason = "INVALID COMMAND: How did ros allow to send me this"
            return response
        
        serial_command = SerialMessage(data=serial_command)

        if self.current_direction in [DCActuatorSrv.Request.DIRECTION_FORWARD, DCActuatorSrv.Request.DIRECTION_BACKWARD] and request.direction != DCActuatorSrv.Request.DIRECTION_STOP:
            # While its moving, can only stop. Cant move again
            response.status = DCActuatorSrv.Response.STATUS_ERROR
            response.reason = "ACTIVE: Already in motion, can only stop"
            return response
        
        self.current_direction = request.direction
        
        if request.direction == DCActuatorSrv.Request.DIRECTION_STOP:
            self.serial_in.publish(serial_command)
            response.status = DCActuatorSrv.Response.STATUS_CANCELLING
            response.reason = "MANUAL STOP:"
            self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
            return response
        
        self.serial_in.publish(serial_command)
        try: 
            serial_response = self.serial_data.get(timeout=self.serial_timeout)
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.serial_timeout} s")
            response.status = DCActuatorSrv.Response.STATUS_ERROR
            response.reason = "TIMEOUT: Failed to get serial response"
            self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
            return response
        
        if serial_response.startswith("ACK:"):
            self.logger.info(f"Got acknowledgment. Executing motion: {serial_response}")

        try: 
            serial_response = self.serial_data.get(timeout=self.serial_timeout)
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.serial_timeout} s")
            response.status = DCActuatorSrv.Response.STATUS_ERROR
            response.reason = "TIMEOUT: Failed to get serial response"
            self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
            return response
                
        if serial_response == "ACK:S":
            response.status = DCActuatorSrv.Response.STATUS_CANCELLED
            response.reason = "MANUAL STOP: cancelled"
            self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
            return response
        
        if not serial_response.startswith("LIMIT:"):
            response.status = DCActuatorSrv.Response.STATUS_ERROR
            response.reason = "Some Unknown error"
            self.logger.error(f"Unknown error: {serial_response}")
            self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
            return response

        self.logger.info("Operation completed successfully")
        response.status = DCActuatorSrv.Response.STATUS_COMPLETED
        self.current_direction = DCActuatorSrv.Request.DIRECTION_STOP
        return response

    def handle_serial_out(self, data: SerialMessage):
        self.device_connected = bool(data.state)
        if not data.data.startswith(SerialBand.DC_ACTUATOR.value):
            self.logger.warning(f"Got invalid: {data.data}")
            return
        self.serial_data.put(data.data.removeprefix(SerialBand.DC_ACTUATOR.value))
    
def main(args=None):
    rclpy.init(args=args)
    node = DCActuator()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
