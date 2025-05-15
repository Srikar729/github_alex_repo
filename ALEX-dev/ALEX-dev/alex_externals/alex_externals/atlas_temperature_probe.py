"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 

"""

"""
Document: https://files.atlas-scientific.com/EZO-TMP-complete-datasheet.pdf
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
from alex_interfaces.msg import TemperatureStatus, SerialMessage
from alex_interfaces.srv import TemperatureCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
from time import sleep
from queue import Queue, Empty as QueueEmpty

class AtlasTemperatureProbe(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.job_id = None
        self.device_connected = False
        # NOTE [v] maxsize = 2 because it response with `*OK` after reading
        self.serial_data: Queue[str] = Queue(maxsize=2)
        self.serial_timeout:int = 5

        self.set_subscribers()
        self.set_publishers()
        self.set_services()

        self.service_client()
        self.configure_device()

        self.logger.info(f"{node_name} started running")
    
    def set_subscribers(self):
        self.serial_out = self.create_subscription(SerialMessage, "from_serial", self.handle_serial_out , 10, callback_group=ReentrantCallbackGroup())

    def set_publishers(self):
        self.pub_to_serial = self.create_publisher(SerialMessage, "to_serial", 10)
        self.pub_temperature_reading = self.create_publisher(TemperatureStatus, "temperature_reading", 1)

    def set_services(self):
        self.create_service(TemperatureCommand, "start_temperature_reading", self.start_temperature_reading )

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
        # NOTE: Once Done
        self.continous_mode(interval=1)
    # ----- Serial Device setup:END -----

    # MARK: Control Logic
    def serial_empty_catcher(function):
        def wrapper(self, *args, **kwargs):
            try:
                result = function(self, *args, **kwargs)
                return result
            except QueueEmpty:
                self.logger.warning(f"Failed to get any response in {self.serial_timeout} s")
                return None
        return wrapper
    
    def serial_float_catcher(function):
        def wrapper(self, *args, **kwargs):
            try:
                result = function(self, *args, **kwargs)
                return result
            except ValueError:
                self.logger.warning(f"Failed to convert to float")
                return None
        return wrapper

    @serial_empty_catcher
    @serial_float_catcher
    def single_read(self):
        # Clear the queue
        with self.serial_data.mutex:
            self.serial_data.queue.clear()
        self.pub_to_serial.publish(SerialMessage(data="R"))
        serial_data = self.serial_data.get(timeout=self.serial_timeout)
        match serial_data:
            case "*ER":
                return None
            case "*OK":
                self.logger.info("Got unexpected `OK`, Should get data at there")
                return None
            case _:
                # Hope I got correct float data
                pass
        temperature_reading = float(serial_data)
        serial_data = self.serial_data.get(timeout=self.serial_timeout) # to get `*OK`
        return temperature_reading
    
    @serial_empty_catcher
    def continous_mode(self, interval:int = 0):
        """
        interval = 0 - Stop continous mode
        interval = 1 to 99 seconds intervals
        """
        # Clear the queue
        with self.serial_data.mutex:
            self.serial_data.queue.clear
        self.pub_to_serial.publish(SerialMessage(data=f"C,{interval}"))
        for _ in range(3):
            serial_data = self.serial_data.get(timeout=self.serial_timeout)
            match serial_data:
                case "*ER":
                    return False
                case "*OK" if interval > 0:
                    self.logger.info("Started Continous mode")
                    return True
                case "*OK": 
                    self.logger.info("Ended Continous mode")
                    return True
                case _:
                    self.logger.debug("Got unresonable data for my request")
        self.logger.error("Did not get valid response from probe in time")
        return False
    
    def calibration(self, value: float):
        with self.serial_data.mutex:
            self.serial_data.queue.clear
        self.pub_to_serial.publish(SerialMessage(data=f"Cal,{value}"))
        for _ in range(3):
            serial_data = self.serial_data.get(timeout=self.serial_timeout)
            match serial_data:
                case "*ER":
                    return False
                case "*OK":
                    return True
                case _:
                    pass
        return False
    
    def start_temperature_reading(self, request:TemperatureCommand.Request, response:TemperatureCommand.Response):
        if not self.device_connected:
            # TODO: Should handle when reconnected but device give data
            self.logger.info("Temperature probe not connected")
            response.status.status = TemperatureStatus.ERROR
            response.status.reason = "Temperature probe not connected"
            return response
        
        match request.command:
            case TemperatureCommand.Request.COMMAND_CUSTOM_MEASURE:
                if not request.job_id:
                    response.success = False
                    response.status.status = TemperatureStatus.ERROR
                    response.status.reason = "Invalid job id"
                    return response              
                self.job_id = request.job_id
                self.logger.info("Started reading the Temperature data")
                response.success = True
                response.status.status = TemperatureStatus.WORKING
                response.status.reason = "STARTED"
            case TemperatureCommand.Request.COMMAND_CUSTOM_STOP:
                self.job_id = None
                self.logger.info("Stopped reading the Temperature data")
                response.success = True
                response.status.status = TemperatureStatus.COMPLETED
                response.status.reason = "STOPPED"
            case TemperatureCommand.Request.COMMAND_CALIBRATE:
                response.status.job_id = request.job_id
                response.success = self.calibration(request.value)
                response.status.status = TemperatureStatus.COMPLETED
            case _:
                response.status.job_id = request.job_id
                response.success = False
                response.status.status = TemperatureStatus.ERROR
                response.status.reason = "Invalid Command"
        return response

    @serial_float_catcher
    def handle_serial_out(self, data: SerialMessage):
        self.device_connected = bool(data.state)
        self.logger.debug(f"Got serial data {data.data}")
        if self.serial_data.full():
            self.serial_data.get()
        self.serial_data.put(data.data)
        msg = TemperatureStatus(status=TemperatureStatus.COMPLETED)
        if not self.job_id:
            return
        if not self.device_connected:
            msg.status = TemperatureStatus.ERROR
            msg.reason = "Device Disconnected"
        msg.job_id = self.job_id
        msg.value.value = float(data.data)
        msg.value.unit  = "C" # This is the default unit
        self.pub_temperature_reading.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AtlasTemperatureProbe()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
