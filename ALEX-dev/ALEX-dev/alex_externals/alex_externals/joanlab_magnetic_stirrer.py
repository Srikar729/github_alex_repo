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
from rclpy.time import Duration
from rclpy.logging import LoggingSeverity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.msg import SerialMessage
from alex_interfaces.msg import MagneticStirrer
from alex_interfaces.srv import MagneticStirrerCommand
# Python Import
from time import sleep
from queue import Queue, Empty as QueueEmpty
# Utilities Import
from alex_utilities.common_utilities import change_case

class JoanlabMagneticStirrer(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.initialize()
        self.parameters()
        self.set_publisher()
        self.set_subscriber()
        self.set_service()
        self.service_client()
        self.configure_device()

    def parameters(self,):
        self.declare_parameter('joanlab_serial_timeout', 10)
        self.serial_timeout:int = self.get_parameter('joanlab_serial_timeout').value

    def initialize(self):
        self.serial_data: Queue[str] = Queue()
        self.device_connected = False

    def set_publisher(self):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_magnetic_stirrer_status = self.create_publisher(MagneticStirrer, "magnetic_stirrer_status", 10)

    def set_subscriber(self):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())

    def set_service(self):
        self.srv_stir  = self.create_service(MagneticStirrerCommand, "start_magnetic_stirrer", self.control_stirrer, callback_group=ReentrantCallbackGroup())
        
    # MARK: Serial Device setup
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

    def ros_publisher_decorator(func):
        def wrapper(self, request: MagneticStirrerCommand.Request , response: MagneticStirrerCommand.Response):
            result: MagneticStirrerCommand.Response = func(self, request, response)
            result.status.job_id = request.job_id
            result.status.rpm = request.rpm
            # TODO: Handle the type conversion
            result.status.timer = request.timer
            result.status.temperature = request.temperature
            self.pub_magnetic_stirrer_status.publish(result.status)
            return result
        return wrapper

    # MARK: Control
    @ros_publisher_decorator
    def control_stirrer(self, request: MagneticStirrerCommand.Request, response: MagneticStirrerCommand.Response):
        self.logger.info(f"Received request: timer={request.timer.value}, unit={request.timer.unit}")

        available_timer_units = {
            "second": 1,
            "minute": 60,
            "hour"  : 3600,
        }

        if request.timer.value <= 0:
            response.status.reason = "Timer value should be more than 1"
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response
        elif available_timer_units.get(request.timer.unit) is None:
            response.status.reason = "Invalid timer units, can handle second, minute, hour"
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response

        s_timer = request.timer.value * available_timer_units.get(request.timer.unit)

        request_duration = Duration(seconds=s_timer)
        serial_commands = {
            "start": "r21",
            "stop" : "r20"
        }
        serial_command = SerialMessage()
        serial_command.data = serial_commands["start"]
        self.serial_in.publish(serial_command)
        try: 
            serial_response = self.serial_data.get(timeout=self.serial_timeout)
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.serial_timeout} s")   
            response.status.status = MagneticStirrer.ERROR
            response.status.reason = "TIMEOUT: Failed to get serial response"
            return response

        if not serial_response.startswith("ACK:"):
            self.logger.error(f"Invalid response from serial device: {serial_response}")
            response.status.status = MagneticStirrer.ERROR
            response.status.reason = f"Invalid response from serial device: {serial_response}"
            return response

        timer_start = self.get_clock().now()
        self.logger.info(f"Got acknowledgment. Executing stir: {serial_response}")

        while rclpy.ok():
            remaining_runtime = request_duration.nanoseconds - (self.get_clock().now() - timer_start).nanoseconds
            remaining_runtime /= 1e9 # converting to seconds
            sleep(1)
            if remaining_runtime < 0:
              break
        else: 
            response.status.status = MagneticStirrer.ERROR
            response.status.reason = f"Something Went wrong"
            return response
        
        self.logger.info(f"Timer completed. Executing stop command")
        serial_command = SerialMessage()
        serial_command.data = serial_commands["stop"]
        self.serial_in.publish(serial_command)
        try: 
            serial_response = self.serial_data.get(timeout=self.serial_timeout)
            self.logger.info(f"Got acknowledgment. Executing stir: {serial_response}")
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.serial_timeout} s")   
            response.status.status = MagneticStirrer.ERROR
            response.status.reason = "TIMEOUT: Failed to get serial response"
            return response
        response.status.status = MagneticStirrer.COMPLETED
        response.status.reason = f"Stop command sent successfully"
        return response
        
    def handle_serial_out(self, data: SerialMessage):
        self.device_connected = bool(data.state)
        if not data.data.startswith("ACK:"):
            return
        self.serial_data.put(data.data)

def main(args=None):
    rclpy.init(args=args)
    node = JoanlabMagneticStirrer()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        node.get_logger().info("Node destroyed")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
