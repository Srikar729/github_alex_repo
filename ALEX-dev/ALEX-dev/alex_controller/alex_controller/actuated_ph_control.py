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
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from rcl_interfaces.msg import ParameterDescriptor
from alex_interfaces.msg import PHMeterStatus, TemperatureStatus
from alex_interfaces.srv import PHMeterCommand, TemperatureCommand
from alex_interfaces.srv import StepperActuatorCommand, MeasurePhCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
import asyncio
from time import sleep
from collections import deque

class ActuatedPhControlNode(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.initialize()
        self.parameters()
        self.create_service_client()
        self.set_services()
        self.set_subscribers()

        self.logger.info(f"{node_name} started running")
        self.logger.info(f"This pH meter is registered {'with' if self.has_temperature else 'without'} temperature sensor")

    
    def initialize(self,):
        self.ph_readings = deque(maxlen=70)
        self.temperature_readings = deque(maxlen=70)
    
    def parameters(self,):
        self.declare_parameter('max_measuring_delay', value=60,   descriptor=ParameterDescriptor(description="Maximum allowed delay for ph measurement"))
        self.declare_parameter('has_temperature',     value=True, descriptor=ParameterDescriptor(description="Temperature probe connected?"))
        self.declare_parameter('actuator_home',       value=490,  descriptor=ParameterDescriptor(description="Home Position of the actuator"))

        self.max_measuring_delay:int   = self.get_parameter('max_measuring_delay').value
        self.has_temperature:bool       = self.get_parameter('has_temperature').value
        self.actuator_home:int          = self.get_parameter('actuator_home').value

    def create_service_client(self,):
        self.actuator_srv                  = self.create_client(StepperActuatorCommand, "actuator/move_actuator")
        self.start_ph_reading_srv          = self.create_client(PHMeterCommand, "ph/start_ph_reading")
        if not self.has_temperature: return
        self.start_temperature_reading_srv = self.create_client(TemperatureCommand, "temperature/start_temperature_reading")
                         
    def set_services(self,):
        self.create_service(MeasurePhCommand, "measure_ph_value", self.measure_ph_value, callback_group=ReentrantCallbackGroup())
    
    def set_subscribers(self,):
        self.sub_ph_reading          = self.create_subscription(PHMeterStatus, "ph/ph_reading", self.handle_ph_reading, 10, callback_group=ReentrantCallbackGroup())
        self.sub_temperature_reading = self.create_subscription(TemperatureStatus, "temperature/temperature_reading", self.handle_temperature_reading, 10, callback_group=ReentrantCallbackGroup())
    
    def handle_ph_reading(self, data:PHMeterStatus):
        self.logger.debug(f"current ph reading: {data}")
        self.ph_readings.append(data.value.value)
    
    def handle_temperature_reading(self, data: TemperatureStatus):
        self.logger.debug(f"current temperature reading: {data}")
        self.temperature_readings.append(data.value.value)

    async def measure_ph_value(self, request:MeasurePhCommand.Request, response:MeasurePhCommand.Response):
        self.ph_readings.clear()
        if request.delay > self.max_measuring_delay:
            response.reason = "Got delay above threshold"
            return response
        if request.mode not in [MeasurePhCommand.Request.AVERAGING, MeasurePhCommand.Request.LASTVALUE]:
            response.reason = "Got invalid measuring mode"
            return response
        if not self.actuator_srv.service_is_ready():
            response.reason = "Actuator service not ready"
            return response
        if not self.start_ph_reading_srv.service_is_ready():
            response.reason = "ph reading service not ready"
            return response
        if self.has_temperature and not self.start_temperature_reading_srv.service_is_ready():
            response.reason = "temperature reading service not ready"
            return response
        actuator_request = StepperActuatorCommand.Request()
        actuator_request.distance.value = float(self.actuator_home)
        actuator_request.distance.unit  = StepperActuatorCommand.Request.UNIT_MILLIMETER
        linear_actuator_future = self.actuator_srv.call_async(actuator_request)
        await linear_actuator_future
        linear_actuator_response: StepperActuatorCommand.Response = linear_actuator_future.result()
        if linear_actuator_response.status != StepperActuatorCommand.Response.STATUS_COMPLETED:
            response.reason = "Actuator homing failed"
            return response
        self.logger.info("Reached the home point")
        actuator_request = StepperActuatorCommand.Request()
        actuator_request.distance.value = request.distance * -1.0
        actuator_request.distance.unit  = StepperActuatorCommand.Request.UNIT_CENTIMETER
        linear_actuator_future = self.actuator_srv.call_async(actuator_request)
        await linear_actuator_future
        linear_actuator_response: StepperActuatorCommand.Response = linear_actuator_future.result()
        if linear_actuator_response.status != StepperActuatorCommand.Response.STATUS_COMPLETED:
            response.reason = "Actuator movement failed"
            return response
        self.logger.info("Reached the measuring point")
        self.ph_readings.clear()
        # Sending start request to temperature
        if self.has_temperature:
            temperature_request = TemperatureCommand.Request()
            temperature_request.command = TemperatureCommand.Request.COMMAND_CUSTOM_MEASURE
            temperature_request.job_id = request.job_id
            temperature_reading_future = self.start_temperature_reading_srv.call_async(temperature_request)
            await temperature_reading_future
            self.logger.info("Temperature reading started")
        # Sending start request to pH
        ph_request = PHMeterCommand.Request()
        ph_request.command = PHMeterCommand.Request.COMMAND_CUSTOM_MEASURE
        ph_request.job_id = request.job_id
        ph_reading_future = self.start_ph_reading_srv.call_async(ph_request)
        await ph_reading_future
        ph_response: PHMeterCommand.Response = ph_reading_future.result()
        if not ph_response.success:
            response.reason = f"pH reading failed: {ph_response.status.reason}"
            return response
        self.logger.info("Ph meter started")
        sleep(request.delay)
        # Sending stop request to temperature
        if self.has_temperature:
            temperature_request.command = TemperatureCommand.Request.COMMAND_CUSTOM_STOP
            temperature_reading_future = self.start_temperature_reading_srv.call_async(temperature_request)
            await temperature_reading_future
            self.logger.info("Temperature stopped")
        # Sending stop request to pH
        ph_request.command = PHMeterCommand.Request.COMMAND_CUSTOM_STOP
        ph_reading_future = self.start_ph_reading_srv.call_async(ph_request)
        await ph_reading_future
        self.logger.info("Ph meter stopped")
        actuator_request = StepperActuatorCommand.Request()
        actuator_request.distance.value = float(self.actuator_home)
        actuator_request.distance.unit  = StepperActuatorCommand.Request.UNIT_MILLIMETER
        linear_actuator_future = self.actuator_srv.call_async(actuator_request)
        await linear_actuator_future
        self.logger.info("Moving back to previous position")
        if not self.ph_readings:
            response.reason = "Did not get any ph readings"
            return response
        response.success = True
        if request.mode == MeasurePhCommand.Request.AVERAGING:
            response.value = sum(self.ph_readings)/len(self.ph_readings)
        else:
            response.value = self.ph_readings[-1]
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ActuatedPhControlNode()
    # Run the executor within the asyncio event loop
    loop = asyncio.get_event_loop()
    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    loop.run_until_complete(rclpy.spin(node, executor=executor))
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
