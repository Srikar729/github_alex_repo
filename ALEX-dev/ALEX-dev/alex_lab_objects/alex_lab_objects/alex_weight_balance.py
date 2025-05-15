from geometry_msgs.msg import Point
from alex_interfaces.msg import Weight
from alex_interfaces.srv import WeightCommand, MeasureWeights
from alex_lab_objects.alex_object import ALEXObject
from alex_lab_objects.alex_container import Container

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class WeightBalance(ALEXObject):
    
    def __init__(self, position: Point, weight_control_srv: Client, logger:RcutilsLogger | None = None, *, actuator_exist:bool = True):
        ALEXObject.__init__(self, position)
        self.logger = logger
        if not self.logger:
            self.logger = lambda a: print(f"[{self.__class__.__name__}]: {a}")
            self.logger.info = lambda a: print(f"[{self.__class__.__name__}] [INFO]: {a}")
            self.logger.error = lambda a: print(f"[{self.__class__.__name__}] [ERROR]: {a}")
            self.logger.fatal = lambda a: print(f"[{self.__class__.__name__}] [FATAl]: {a}")
        self.weight_control_srv = weight_control_srv
        self.actuator_exist = actuator_exist
    
    async def open_door(self):
        if not self.actuator_exist:
            self.logger.info("Actuator not connected, So skipping")
            return True
        if not self.weight_control_srv.service_is_ready():
            raise Exception("Service not ready")
        request = MeasureWeights.Request()
        request.command     = MeasureWeights.Request.COMMAND_DOOR_OPEN
        future = self.weight_control_srv.call_async(request)
        await future
        response:MeasureWeights.Response = future.result()
        if not response.success:
            self.logger.error(f"Failed to open door with reason: {response.reason}")
            raise Exception(response.reason)
        return response.success

    async def close_door(self):
        if not self.actuator_exist:
            self.logger.info("Actuator not connected, So skipping")
            return True
        if not self.weight_control_srv.service_is_ready():
            raise Exception("Service not ready")
        request = MeasureWeights.Request()
        request.command     = MeasureWeights.Request.COMMAND_DOOR_CLOSE
        future = self.weight_control_srv.call_async(request)
        await future
        response:MeasureWeights.Response = future.result()
        if not response.success:
            self.logger.error(f"Failed to measure with reason: {response.reason}")
            raise Exception(response.reason)
        return response.success

    async def measure(self, target: Container, job_id=""):
        if not self.weight_control_srv.service_is_ready():
            raise Exception("Service not ready")
        request = MeasureWeights.Request()
        request.command     = MeasureWeights.Request.COMMAND_MEASURE
        request.weight_mode = WeightCommand.Request.MODE_STABLE
        request.job_id      = job_id
        future = self.weight_control_srv.call_async(request)
        await future
        response:MeasureWeights.Response = future.result()
        if not response.success:
            self.logger.error(f"Measure failed with reason: {response.reason}")
            raise Exception(response.reason)
        return response.result.value.value
