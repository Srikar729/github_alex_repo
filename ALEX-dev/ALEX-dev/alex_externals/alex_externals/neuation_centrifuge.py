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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.msg import SerialMessage, CentrifugeStatus
from alex_interfaces.srv import CentrifugeCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
from time import sleep
from enum import Enum
from threading import Event

class Controls(Enum):
    RPM = 23
    MINUS = 27
    PLUS = 43
    START = 46
    TIME = 29
    P1 = 39
    P2 = 45
    STOP = 30

class CentrifugeControlNode(Node):
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
        self.is_running = Event()

    def set_publisher(self):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_centrifuge_status = self.create_publisher(CentrifugeStatus, "centrifuge_status", 10)
    
    def set_subscriber(self):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())

    def set_service(self):
        # ROS2 service for centrifuge
        self.srv = self.create_service(CentrifugeCommand, 'start_centrifuge', self.control_centrifuge_callback, callback_group=ReentrantCallbackGroup())

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

    @staticmethod
    def breakdown_rpm_units(current_value, target_value):
        difference = target_value - current_value

        thousands = difference // 1000  # Get number of 1000s
        remainder = difference % 1000   # Get remainder after extracting 1000s
        hundreds  = remainder // 100 # Get number of 100s
        
        return thousands, hundreds

    def handle_serial_out(self, data: SerialMessage):
        self.logger.debug(f"Got data: {data.data}")
    
    def send_command(self, sequence: Controls):
        command = f"AT+SEQ={sequence.value}"
        self.serial_in.publish(SerialMessage(data=command))
        self.get_logger().debug(f"Sent: {command}")
    
    def send_commands(self, commands: list[Controls]):
        self.logger.debug(f"{commands=}")
        for command in commands:
            self.send_command(command)
            sleep(1)

    def set_stop_open(self, wait: bool = True):
        self.logger.info("Sending stop command")
        self.send_command(Controls.STOP)
        if wait:
            sleep(15) # TODO: need to modify this time after testing how long it takes for the spining to stop and open the lid
            self.logger.info("Lid should be poped open now")
    
    def set_initial_rpm_500(self):
        commands = [ Controls.RPM ] * 2 + [ Controls.MINUS ] * 5

        self.send_commands(commands)

    def set_rpm(self, rpm: int):
        commands =  [ Controls.RPM ]
        thousands, hundreds = self.breakdown_rpm_units(500, rpm)
        if hundreds:
            hundreds_commands = [ Controls.PLUS ] * hundreds
            commands.extend(hundreds_commands)
        if thousands:
            thousands_commands = [ Controls.RPM ] + [ Controls.PLUS ] * thousands
            commands.extend(thousands_commands)
        self.send_commands(commands)
    
    def ros_publisher_decorator(func):
        def wrapper(self, request: CentrifugeCommand.Request , response: CentrifugeCommand.Response):
            result: CentrifugeCommand.Response = func(self, request, response)
            self.is_running.clear()
            self.set_stop_open(result.status.status!=CentrifugeStatus.CANCELLING) # Stops the centrifuge if not cancelling
            result.status.job_id = request.job_id
            result.status.timer = request.timer
            result.status.rpm = request.rpm
            self.pub_centrifuge_status.publish(result.status)
            return result
        return wrapper
    
    # MARK: Control Logic
    @ros_publisher_decorator
    def control_centrifuge_callback(self, request: CentrifugeCommand.Request, response: CentrifugeCommand.Response):
        self.logger.info(f"Got Request rpm={request.rpm} timer={request.timer.value} {request.timer.unit}")

        if request.timer.value == request.rpm == 0:
            response.success = True
            response.status.reason = "MANUAL STOP: Stopping request accepted."
            response.status.status = CentrifugeStatus.CANCELLING
            self.logger.warning(response.status.reason)
            # Return will trigger the centrifuge stop
            return response
        
        if self.is_running.is_set():
            response.status.reason = "Centrifuge is already running"
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if request.rpm < 500:
            response.status.reason = "Minimum speed is 500 RPM"
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if request.rpm > 4500:
            response.status.reason = "Maximum speed is 4500 RPM"
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response
        self.is_running.set()
        
        available_timer_units = {
            "second": 1,
            "minute": 60,
            "hour"  : 3600,
        }

        timer = request.timer.value * available_timer_units.get(request.timer.unit, 0)
        request_duration = Duration(seconds=timer)
        # timer in minutes
        timer /= 60
        
        if timer <= 0:
            response.status.reason = "Invalid timer units, can handle second, minute, hour"
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if timer > 99:
            response.status.reason = "Timer out of bound. Max time is 99"
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if request.rpm % 100 != 0:
            response.status.reason = "Invalid rpm, must be in factor of 100s"
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response

        self.logger.info("Starting Program 1. Make sure this is pre configured")
        self.send_command(Controls.P1)
        
        self.logger.info("Setting initial speed to 500")
        self.set_initial_rpm_500()
        self.logger.info(f"Setting the {request.rpm=}")
        self.set_rpm(request.rpm)

        timer_start = self.get_clock().now()

        self.send_command(Controls.START)
        while rclpy.ok():
            remaining_runtime = request_duration.nanoseconds - (self.get_clock().now() - timer_start).nanoseconds
            remaining_runtime /= (60 * 1e9) # converting to minutes
            
            self.logger.info(f"Yet to reach the requested duration. Time left: {remaining_runtime:.2f} minutes")
            if remaining_runtime < 0:
                break

            if not self.is_running.is_set():
                response.status.reason = "MANUAL STOP: The centrifuge is stopped manually"
                response.status.status = CentrifugeStatus.CANCELLED
                self.logger.warning(response.status.reason)
                return response

            self.rate.sleep()
        else:
            response.status.reason = "Something went wrong with ROS."
            response.status.status = CentrifugeStatus.ERROR
            self.logger.warning(response.status.reason)
            return response

        self.logger.info("Operation completed successfully")
        response.status.status = CentrifugeStatus.COMPLETED
        response.success = True
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = CentrifugeControlNode()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
