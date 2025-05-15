"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 

"""

"""
Document: https://neuralfoundry.sharepoint.com/:b:/s/URP/ER2nOqEnOMFHhkEUUI6ECjcBQSP03izYfidaA2fYXU46ZQ?e=to04Mf
"""

# ROS Import
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Duration
from rclpy.logging import LoggingSeverity
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from alex_interfaces.msg import SerialMessage, MagneticStirrer
from alex_interfaces.srv import MagneticStirrerCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
from enum import Enum
from time import sleep
from threading import Event

class PREFIX(Enum):
    COMMAND  = 0xFE
    RESPONSE = 0xFD

    INVALID  = "INVALID"

    @classmethod
    def _missing_(cls, _):
        return cls.INVALID

class INSTRUCTIONCODE(Enum):
    # Instruction
    HELLO       = 0xA0
    INFO        = 0xA1
    STATUS      = 0xA2
    # Control
    MOTOR       = 0xB1
    TEMPERATURE = 0xB2

    INVALID     = "INVALID"

    @classmethod
    def _missing_(cls, _):
        return cls.INVALID

class ScitekMagneticStirrer(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        # self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.initilize()
        self.parameters()
        self.set_publisher()
        self.set_subscriber()
        self.set_service()
        self.service_client()
        self.configure_device()
        
        self.timer = self.create_timer(1, self.initial_hello_send)
        
    def initial_hello_send(self,):
        self.timer.cancel()
        result = self.send_hello()
        self.logger.info(f"Initial Hello: {result}")

        result = self.send_info()
        self.logger.info(f"Initial Info: {result}")

        result = self.send_status()
        self.logger.info(f"Initial Status: {result}")

    def initilize(self):
        self.hello_response = False
        self.wait_hello_thread = Event()

        self.info_response = None
        self.wait_info_thread = Event()

        self.status_response = None
        self.wait_status_thread = Event()

        self.stirrer_response = False
        self.wait_stirrer_thread = Event()

        self.temperature_response = False
        self.wait_temperature_thread = Event()

        self.rate = self.create_rate(0.20) # 5 seconds

    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('scitek_serial_timeout', 10)
        self.declare_parameter('speed_tolerance', 5)
        # get ros 2 parameters
        self.scitek_serial_timeout:int = self.get_parameter('scitek_serial_timeout').value
        self.speed_tolerance:int       = self.get_parameter('speed_tolerance').value

    def set_publisher(self):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_magnetic_stirrer_status = self.create_publisher(MagneticStirrer, "magnetic_stirrer_status", 10)
    
    def set_subscriber(self,):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())

    def set_service(self):
        self.srv_magnetic_stirrer = self.create_service(MagneticStirrerCommand, "start_magnetic_stirrer", self.start_magnetic_stirrer)
    
    # MARK: Serial Setup
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
    def _check_sum(data:list[int]):
        """Calculate checksum by summing the data bytes."""
        return sum(data) & 0xFF  # Only keep the last 8 bits
    
    def send_command(self, command:int, params:list[int]=[]):
        command_data = [command]
        while len(params) < 3:
            params.append(0x00)
        command_data.extend(params)
        check_sum = self._check_sum(command_data)
        command_data.append(check_sum)
        command_data.insert(0, PREFIX.COMMAND.value)

        data = bytearray(command_data).hex()
        self.logger.debug(f"Sending data: {data}")

        self.serial_in.publish(SerialMessage(data=data))
    
    def retry_send(threshold:int=3):
        def decorator(func):
            def wrapper(self, *args, **kwargs):
                retry_threshold = threshold
                for retry_count in range(retry_threshold):
                    result = func(self, *args, **kwargs)
                    if result is not None: 
                        break
                    self.logger.warning(f"Failed to get result for `{func.__name__}` trying again... {retry_count+1}/{retry_threshold}")
                return result
            return wrapper
        return decorator

    @retry_send(threshold=1)
    def send_hello(self,):
        self.wait_hello_thread.clear()
        self.send_command(INSTRUCTIONCODE.HELLO.value)
        success = self.wait_hello_thread.wait(self.scitek_serial_timeout)
        if not success:
            self.logger.warning(f"Failed to get response for `send_hello` in {self.scitek_serial_timeout}s")
            return 
        return self.hello_response
    
    @retry_send(threshold=1)
    def send_info(self,):
        self.wait_info_thread.clear()
        self.send_command(INSTRUCTIONCODE.INFO.value)
        success = self.wait_info_thread.wait(self.scitek_serial_timeout)
        if not success:
            self.logger.warning(f"Failed to get response for `send_info` in {self.scitek_serial_timeout}s")
            return 
        return self.info_response

    @retry_send()
    def send_status(self,):
        self.wait_status_thread.clear()
        self.send_command(INSTRUCTIONCODE.STATUS.value)
        success = self.wait_status_thread.wait(self.scitek_serial_timeout)
        if not success:
            self.logger.warning(f"Failed to get response for `send_status` in {self.scitek_serial_timeout}s")
            return 
        return self.status_response

    @retry_send()
    def send_stirrer(self, rpm: int):
        self.wait_stirrer_thread.clear()
        speed_high = (rpm >> 8) & 0xFF
        speed_low  = rpm & 0xFF
        self.send_command(INSTRUCTIONCODE.MOTOR.value, [speed_high, speed_low])
        success = self.wait_stirrer_thread.wait(self.scitek_serial_timeout)
        if not success:
            self.logger.warning(f"Failed to get response for `send_stirrer` in {self.scitek_serial_timeout}s")
            return
        return self.stirrer_response

    @retry_send()
    def send_temperature(self, value_c: float):
        self.wait_temperature_thread.clear()
        value = int(value_c * 10)
        temp_high = (value >> 8) & 0xFF
        temp_low = value & 0xFF
        self.send_command(INSTRUCTIONCODE.TEMPERATURE.value, [temp_high, temp_low])
        success = self.wait_temperature_thread.wait(self.scitek_serial_timeout)
        if not success:
            self.logger.warning(f"Failed to get response for `send_temperature` in {self.scitek_serial_timeout}s")
            return
        return self.temperature_response

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

    # MARK: Service
    @ros_publisher_decorator
    def start_magnetic_stirrer(self, request:MagneticStirrerCommand.Request, response:MagneticStirrerCommand.Response):
        self.logger.info(f"Request recived: RPM={request.rpm} timer={request.timer.value} {request.timer.unit} temperature={request.timer.value} {request.timer.unit}")
        
        available_timer_units = {
            "second": 1,
            "minute": 60,
            "hour"  : 3600,
        }

        timer = request.timer.value * available_timer_units.get(request.timer.unit, 0)
        request_duration = Duration(seconds=timer)
        timer_start = None

        hello_result = self.send_hello()
        if not hello_result:
            response.status.reason = "Failed to get hello from the device"
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response

        if not (100 <= request.rpm <= 1500):
            response.status.reason = "RPM should be within 100 to 1500"
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if request.temperature.value:
            temp_result = self.send_temperature(request.temperature.value)
            if not temp_result:
                response.status.reason = "Failed to set temperature"
                response.status.status = MagneticStirrer.ERROR
                self.logger.warning(response.status.reason)
                return response
        
        stirrer_result = self.send_stirrer(request.rpm)
        if not stirrer_result:
            response.status.reason = "Failed to set motor speed"
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response

        # TODO: Start the motor. Click the knob

        info_result = self.send_info()
        if not info_result.get("Stirrer Status"):
            response.status.reason = "The Stirrer status is not ON. Failed to start the motor"
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response
        

        while rclpy.ok():
            #TODO: Do we need cancel cancel for this service?
            status_result = self.send_status()
            if not status_result:
                response.status.reason = "Unable to get current status"
                response.status.status = MagneticStirrer.ERROR
                self.logger.warning(response.status.reason)
                return response
            
            set_rpm = status_result.get("Set Speed (RPM)")
            if set_rpm != request.rpm:
                response.status.reason = "The set speed is not same as the requested speed. Some manual interference detected"
                response.status.status = MagneticStirrer.ERROR
                self.logger.warning(response.status.reason)
                return response
            
            real_rpm = status_result.get("Real Speed (RPM)")
            if (request.rpm - self.speed_tolerance) >= real_rpm <= (request.rpm - self.speed_tolerance):
                self.logger.info(f"Currently not in required rpm {real_rpm=}")
                continue

            if not timer_start:
                timer_start = self.get_clock().now()
                continue
            
            remaining_runtime = request_duration.nanoseconds - (self.get_clock().now() - timer_start).nanoseconds
            remaining_runtime /= (60 * 1e9) # converting to minutes
            if remaining_runtime < 0:
                break

            self.logger.info(f"Yet to reach the requested duration. Time left: {remaining_runtime:.2f} minutes")

            self.rate.sleep()
        else:
            response.status.reason = "Something went wrong with ROS."
            response.status.status = MagneticStirrer.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        # TODO: Stop the motor. Click the knob
        self.logger.info("Operation completed successfully")
        response.status.status = MagneticStirrer.COMPLETED
        response.success = True
        self.send_stirrer(100)
        return response

    def _parse_hello_response(self, response: list[int]):
        self.hello_response = [True, False][response[0]]
        self.logger.debug(f"{self.hello_response=}")
        self.wait_hello_thread.set()

    def _parse_info_response(self, response: list[int]):
        """Parse the device information response (CMD_INFO)."""
        response_range = range(len(response))

        info = {}

        if 0 in response_range:
            # TODO BUG: Check with mode C, There was some issue last time
            modes = [None, "A", "B", "C"][response[0]]
            info["Mode"] = modes
        if 1 in response_range:
            stirrer_status = [True, False][response[1]]
            info["Stirrer Status"] = stirrer_status
        if 2 in response_range:
            temperature_status = [True, False][response[2]]
            info["Temperature Status"] = temperature_status
        if 3 in response_range:
            high_temp = response[3]
            low_temp  = (response[4] if 4 in response_range else 0x00)
            info["Safe Temperature"] = (high_temp << 8) | low_temp
        if 5 in response_range:
            residual_temperature_warning = response[5]
            info["Residual Temp Warning"] = residual_temperature_warning

        self.logger.debug(f"{info=}")
        self.info_response = info
        self.wait_info_thread.set()

    def _parse_status_response(self, response: list[int]):
        """Parse the device status response (CMD_STATUS)."""
        status = {
            "Set Speed (RPM)": int((response[0] << 8) | response[1]),
            "Real Speed (RPM)": int((response[2] << 8) | response[3]),
            "Set Temperature (°C)": int((response[4] << 8) | response[5]),
            "Real Temperature (°C)": int((response[6] << 8) | response[7]),
        }

        self.logger.debug(f"{status=}")
        self.status_response = status
        self.wait_status_thread.set()
    
    def _parse_stirrer_response(self, response: list[int]):
        stirrer = [True, False][response[0]]
        self.logger.debug(f"{stirrer=}")
        self.stirrer_response = stirrer
        self.wait_stirrer_thread.set()
    
    def _parse_temperature_response(self, response: list[int]):
        temperature = [True, False][response[0]]
        self.logger.debug(f"{temperature=}")
        self.temperature_response = temperature
        self.wait_temperature_thread.set()

    def handle_serial_data_frame(self, instruction_code: INSTRUCTIONCODE, data_frame: list[int]):
        if instruction_code == INSTRUCTIONCODE.HELLO:
            self._parse_hello_response(data_frame)
        elif instruction_code == INSTRUCTIONCODE.INFO:
            self._parse_info_response(data_frame)
        elif instruction_code == INSTRUCTIONCODE.STATUS:
            self._parse_status_response(data_frame)
        elif instruction_code == INSTRUCTIONCODE.MOTOR:
            self._parse_stirrer_response(data_frame)
        elif instruction_code == INSTRUCTIONCODE.TEMPERATURE:
            self._parse_temperature_response(data_frame)
        else:
            self.logger.warning("Got invalid instruction_code, How is that even possible?")

    def handle_serial_out(self, data:SerialMessage):
        self.logger.debug(f"Got data: {data.data}")
        data = bytearray.fromhex(data.data)
        if not len(data) > 2:
             self.logger.info("Invalid data length. Unable to parse")
             return
        prefix, instruction_code, *data_frame, check_sum = data
        prefix = PREFIX(prefix)
        if prefix == PREFIX.INVALID:
            self.logger.error("Got invalid PREFIX")
            return
        if prefix == PREFIX.COMMAND:
            self.logger.debug("Message not for me, Ignore")
            return
        instruction_code = INSTRUCTIONCODE(instruction_code)
        if instruction_code == INSTRUCTIONCODE.INVALID:
            self.logger.warning(f"Got invalid instruction_code:={hex(instruction_code)}")
            return
        
        if not len(data_frame) > 2:
           self.logger.warning(f"Data frame should always have more than 3 value. But got less than it. Not able to parse")
           return
        
        self.handle_serial_data_frame(instruction_code, data_frame)

def main(args=None):
    rclpy.init(args=args)
    node = ScitekMagneticStirrer()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
