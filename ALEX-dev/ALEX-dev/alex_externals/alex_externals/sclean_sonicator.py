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
from alex_interfaces.msg import SerialMessage, Sonicator
from alex_interfaces.srv import SonicatorCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.external_constants import SerialBand
# Python Import
from time import sleep

class SCleanSonicator(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        # self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.parameters()
        self.set_publisher()
        self.set_subscriber()
        self.set_service()
        self.service_client()
        self.configure_device()

        self.rate = self.create_rate(0.5)
        self.maintain_temperature = self.default_temperature
        self.current_temperature  = 0.0
        self.temperature_wait_timer = self.create_timer(self.temperature_wait_timeout_min, self.heater_shutoff)

        self.temperature_logger = None
        
    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('default_temperature', 25.0)
        self.declare_parameter('temperature_threshold', 2.5)
        self.declare_parameter('temperature_wait_timeout_min', 300.0)
        self.declare_parameter('heating_timeout', 5.0)
        # get ros 2 parameters
        self.default_temperature:float = self.get_parameter('default_temperature').value
        self.temperature_threshold:float = self.get_parameter('temperature_threshold').value
        self.temperature_wait_timeout_min:float = self.get_parameter('temperature_wait_timeout_min').value
        self.heating_timeout:float = self.get_parameter('heating_timeout').value

    def set_publisher(self):
        self.serial_in = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_sonicator_status = self.create_publisher(Sonicator, "sonicator_status", 10)
    
    def set_subscriber(self,):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())

    def set_service(self):
        self.srv_sonication  = self.create_service(SonicatorCommand, "start_sonication",  self.ros_publisher_decorator(self.start_sonication_heating))
        self.srv_heating     = self.create_service(SonicatorCommand, "start_heating",     lambda req, resp: self.start_sonication_heating(req, resp, heating_service=True))
    
    # ---- Serial Device setup -----
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
    
    def send_command(self, command: int, status: bool ):
        data = ""
        if command == Sonicator.COMMAND_HEAT:
            data += "SH:"
        elif command == Sonicator.COMMAND_SONICATE:
            data += "SS:"
        data += str(int(status))
        self.logger.debug(f"Sending data: {data}")
        self.serial_in.publish(SerialMessage(data=data))
    
    def heater_shutoff(self):
        self.send_command(Sonicator.COMMAND_HEAT, False)
        self.logger.error(f"Did not get current temperature without the timeout({self.temperature_wait_timeout_min} min), Shuting down heater")

    @staticmethod
    def fahrenheit_to_celsius(fahrenheit: float):
        return (fahrenheit - 32) * 5/9

    def ros_publisher_decorator(self, func):
        def wrapper(request: SonicatorCommand.Request , response: SonicatorCommand.Response):
            result: SonicatorCommand.Response = func(request, response)
            result.status.job_id = request.job_id
            result.status.timer = request.timer
            result.status.temperature.value = self.current_temperature
            result.status.temperature.unit = "C"
            self.pub_sonicator_status.publish(result.status)
            self.send_command(Sonicator.COMMAND_SONICATE, False) # Make sure its off while returning
            self.maintain_temperature = self.default_temperature
            return result
        return wrapper
    
    def start_sonication_heating(self, request:SonicatorCommand.Request, response:SonicatorCommand.Response, heating_service:bool = False):
        self.logger.info(f"Got Request {heating_service=}: temperature={request.temperature.value} {request.temperature.unit} timer={request.timer.value} {request.timer.unit}")
        
        if request.timer.value <= 0:
            if heating_service:
                request.timer.value = self.heating_timeout
                request.timer.unit = "minute"
            else:
                response.status.reason = "Timer value should be more than 1"
                response.status.status = Sonicator.ERROR
                self.logger.warning(response.status.reason)
                return response

        available_timer_units = {
            "second": 1,
            "minute": 60,
            "hour"  : 3600,
        }

        timer = request.timer.value * available_timer_units.get(request.timer.unit, 0)
        
        if timer <= 0:
            response.status.reason = "Invalid timer units, can handle second, minute, hour"
            response.status.status = Sonicator.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if request.temperature.value <= 0:
            response.status.reason = "Temperature value should be more than 1"
            response.status.status = Sonicator.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        available_temperature_units = {
            "C": lambda data : data,
            "F": self.fahrenheit_to_celsius
        }

        temperature = available_temperature_units.get(request.temperature.unit, lambda _: None)(request.temperature.value)

        if not temperature:
            response.status.reason = "Invalid temperature unit, can handle  C, F"
            response.status.status = Sonicator.ERROR
            self.logger.warning(response.status.reason)
            return response
                
        request_duration = Duration(seconds=timer)

        if not heating_service:
            self.send_command(Sonicator.COMMAND_SONICATE, True)
        self.maintain_temperature = temperature
        timer_start = self.get_clock().now()
        
        while rclpy.ok():
            #TODO: Do we need cancel cancel for this service?

            remaining_runtime = request_duration.nanoseconds - (self.get_clock().now() - timer_start).nanoseconds
            remaining_runtime /= (60 * 1e9) # converting to minutes
            if remaining_runtime < 0:
                break
            
            if (
                heating_service and 
                (self.maintain_temperature - self.temperature_threshold <= self.current_temperature <= self.maintain_temperature + self.temperature_threshold)
                ):
                self.logger.info("The requested temperatue reached.")
                response.success = True
                response.status.status = Sonicator.COMPLETED

                response.status.temperature.unit = "C"
                response.status.temperature.value = self.current_temperature
                
                response.status.timer.value = float((self.get_clock().now() - timer_start).to_msg().sec)
                response.status.timer.unit  = "second"
                return response

            self.logger.info(f"Yet to reach the requested duration. Time left: {remaining_runtime:.2f} minutes")

            self.rate.sleep()
        else:
            response.status.reason = "Something went wrong with ROS."
            response.status.status = Sonicator.ERROR
            self.logger.warning(response.status.reason)
            return response
        
        if heating_service:
            response.status.reason = "Failed to reach the required temperature in time"
            self.logger.info(response.status.reason)
            response.status.status = Sonicator.ERROR
            return response
        
        self.logger.info("Operation completed successfully")
        response.status.status = Sonicator.COMPLETED
        response.success = True
        return response
    
    def handle_temperature_control(self, temperature: float):
        if temperature < 0.0:
            self.logger.error(f"Temperature value is below 0. Verify the sensor {temperature=}")
            return
        self.current_temperature = temperature       
        if temperature < self.maintain_temperature - self.temperature_threshold:
            self.send_command(Sonicator.COMMAND_HEAT, True)
            temperature_logger = f"Required Temperature={self.maintain_temperature} > {round(temperature)} , Switching `ON` Heater"
        elif temperature > self.maintain_temperature + self.temperature_threshold:
            self.send_command(Sonicator.COMMAND_HEAT, False)
            temperature_logger = f"Required Temperature={self.maintain_temperature} < {round(temperature)} , Switching `OFF` Heater"
        else:
            temperature_logger = "Within the right temperature"
        # Reset temperature wait timer,
        if self.temperature_logger != temperature_logger:
            self.temperature_logger = temperature_logger
            self.logger.info(self.temperature_logger)
        self.temperature_wait_timer.reset()

    def handle_serial_out(self, serial_data:SerialMessage):
        if not serial_data.data:
            return
        self.logger.debug(f"Got data: {serial_data.data}")

        if serial_data.data.startswith(SerialBand.ERROR.value):
            self.logger.error(f"Got error from serial: {serial_data.data}")
            return
        
        elif serial_data.data.startswith(SerialBand.SONICATOR_TEMPERATURE.value):
            # Handled below
            pass
        else:
            self.logger.warning(f"Got Invalid temperature value: {serial_data.data}")
            return
        
        value = serial_data.data.removeprefix(SerialBand.SONICATOR_TEMPERATURE.value)

        try: 
            temperature = float(value)
            self.handle_temperature_control(temperature)
        except ValueError as e:
            self.logger.warning(f"Got invalid temperature value: {value}")

def main(args=None):
    rclpy.init(args=args)
    node = SCleanSonicator()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
