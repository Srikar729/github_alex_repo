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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.msg import SerialMessage
from alex_interfaces.msg import Homogenizer
from alex_interfaces.srv import HomogenizerCommand
# Python Import
from time import sleep
from queue import Queue, Empty as QueueEmpty
# Utilities Import
from alex_utilities.common_utilities import change_case

class WiggensHomogenizer(Node):
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
        self.declare_parameter('homogenizer_serial_timeout', 10)
        self.declare_parameter('homogenizer_speed_limits', [1_000, 30_000])
        self.declare_parameter('acc_delay', 5)
        self.declare_parameter('dec_delay', 5)
        self.declare_parameter('acc_speed_step', 10)
        self.declare_parameter('dec_speed_step', 10)
        self.serial_timeout:int = self.get_parameter('homogenizer_serial_timeout').value
        self.speed_limits:list[int, int] = self.get_parameter('homogenizer_speed_limits').value
        self.acc_step = self.get_parameter('acc_speed_step').value
        self.dec_step = self.get_parameter('dec_speed_step').value
        self.acc_delay = self.get_parameter('acc_delay').value
        self.dec_delay = self.get_parameter('dec_delay').value

    def initialize(self):
        self.serial_data: Queue[str] = Queue()
        self.device_connected = False

    def set_publisher(self):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_homogenizer_status = self.create_publisher(Homogenizer, "homogenizer_status", 10)

    def set_subscriber(self):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=MutuallyExclusiveCallbackGroup())

    def set_service(self):
        self.srv_mix = self.create_service(HomogenizerCommand, "start_homogenizer", self.control_homogenizer)
        
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
        def wrapper(self, request: HomogenizerCommand.Request , response: HomogenizerCommand.Response):
            result: HomogenizerCommand.Response = func(self, request, response)
            result.status.job_id = request.job_id
            result.status.rpm = request.rpm
            result.status.timer = request.timer
            self.pub_homogenizer_status.publish(result.status)
            return result
        return wrapper

    # MARK: Control Methods
    @ros_publisher_decorator
    def control_homogenizer(self, request: HomogenizerCommand.Request, response: HomogenizerCommand.Response):
        self.logger.info(f"Received request:rpm={request.rpm}, timer={request.timer.value}, unit={request.timer.unit}")

        available_timer_units = {"second": 1, "minute": 60, "hour": 3600}

        s_timer = request.timer.value * available_timer_units.get(request.timer.unit)

        if not self.speed_limits[0] <= request.rpm <= self.speed_limits[1]: # percentage limits.
            return self._respond_with_error(response=response, reason="RPM value should be within 1_000 to 30_000")
        
        speed_percentage = self.rpm_to_percentage(request.rpm, ll_rpm=self.speed_limits[0], ul_rpm=self.speed_limits[1])
        self.logger.info(f"Speed percentage is: {speed_percentage}")
        
        if s_timer <= 0: return self._respond_with_error(response=response, reason="Timer value should be more than 1 Second")
        elif available_timer_units.get(request.timer.unit) is None:
            return self._respond_with_error(response=response, reason="Invalid timer units, can handle second, minute, hour")
        
        # TODO: Need to change these commands
        self.serial_commands = {"start": "AT+POWER=ON", "stop": "AT+POWER=OFF", "speed": "AT+SPEED="}

        self.logger.info("Executing start command")
        if not self._send_serial_command(command=self.serial_commands["start"], response=response):
            return response
        
        # Acceleration
        if not self.ramp_speed(
            start=0,
            target=speed_percentage,
            step=self.acc_step,
            delay=self.acc_delay,
            response=response
        ):
            return response
        # ------------------END ------------------

        request_duration = Duration(seconds=s_timer)
        timer_start = self.get_clock().now()

        while rclpy.ok():
            remaining_runtime = request_duration.nanoseconds - (self.get_clock().now() - timer_start).nanoseconds
            remaining_runtime //= 1e9 # converting to seconds
            self.logger.info(f"Timer Running. Remaining time -> {remaining_runtime}", throttle_duration_sec=1)
            sleep(1)
            if remaining_runtime <= 0:
              break
        else: 
            response.status.status = Homogenizer.ERROR
            response.status.reason = f"Something Went wrong"
            return response
        
        self.logger.info(f"Timer completed. Executing Deceleration")

        # Deceleration
        if not self.ramp_speed(
            start=speed_percentage,
            target=0,
            step=self.dec_step,
            delay=self.dec_delay,
            response=response
        ):
            return response
        # ------------------END ------------------

        self.logger.info(f"Deceleration completed. Executing stop command")
        if not self._send_serial_command(command=self.serial_commands["stop"], response=response):
            return response
        
        response.success = True
        response.status.status = Homogenizer.COMPLETED
        response.status.reason = f"Stop command sent successfully"
        return response
        
    def handle_serial_out(self, data: SerialMessage):
        self.device_connected = bool(data.state)
        if not data.data.startswith("ATR+"):
            return
        self.serial_data.put(data.data)

    # MARK: Helper Methods
    def _send_serial_command(self, command: str, response:HomogenizerCommand.Response) -> bool:
        serial_command = SerialMessage(data=command)
        self.serial_in.publish(serial_command)
        try: 
            serial_response = self.serial_data.get(timeout=self.serial_timeout)
            self.logger.info(f"Received serial response: {serial_response}")
            return True
        except QueueEmpty:
            self.logger.warning(f"Failed to get response in {self.serial_timeout} s")   
            response.status.status = Homogenizer.ERROR
            response.status.reason = "TIMEOUT: Failed to get serial response"
            return False

    def _respond_with_error(self, response: HomogenizerCommand.Response, reason: str):
        response.status.status = Homogenizer.ERROR
        response.status.reason = reason
        self.logger.warning(reason)
        return response
    
    def rpm_to_percentage(self, RPM, ll_rpm=1_000, ul_rpm=30_000):
        distance_from_start = RPM - ll_rpm
        portion = distance_from_start / (ul_rpm - ll_rpm)
        scaled_percentage = portion * 99
        result = scaled_percentage + 1
        return round(result)
    
    def ramp_speed(self, start: int, target: int, step: int, delay: float, response: HomogenizerCommand.Response) -> bool:
        speed = start

        # Determine ramp direction: +1 (acceleration) or -1 (deceleration)
        direction = 1 if target > start else -1

        while speed != target:
            next_speed = speed + direction * step

            # Clamp next_speed to not overshoot target
            if (direction == 1 and next_speed > target) or (direction == -1 and next_speed < target):
                next_speed = target

            self.logger.info(f"{'Increasing' if direction == 1 else 'Decreasing'} speed -> {next_speed}")
            if not self._send_serial_command(command=self.serial_commands["speed"] + str(next_speed), response=response):
                return False

            sleep(delay)
            speed = next_speed

        return True

def main(args=None):
    rclpy.init(args=args)
    node = WiggensHomogenizer()
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
