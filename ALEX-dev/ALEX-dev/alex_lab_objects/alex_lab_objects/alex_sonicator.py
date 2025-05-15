from geometry_msgs.msg import Point
from alex_interfaces.msg import QuantityValue
from alex_interfaces.srv import SonicatorCommand
from alex_lab_objects.alex_object import ALEXObject

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class Sonicator(ALEXObject):
    
    def __init__(self, position: Point, sonicator_control_srv: Client, heating_control_srv: Client, *, should_raise: bool = True, logger:RcutilsLogger | None = None):
        ALEXObject.__init__(self, position)
        self.logger = lambda a: logger.info(f"[{self.__class__.__name__}]: "+f"{a}") if logger else print(f"[{self.__class__.__name__}]: "+f"{a}")
        self.sonicator_control_srv = sonicator_control_srv
        self.heating_control_srv   = heating_control_srv
        self.should_raise = should_raise
    
    async def sonicate(self, timer: float, temperature: QuantityValue, job_id=""):
        if not self.sonicator_control_srv.service_is_ready():
            raise Exception("Sonicator Service is not ready")
        request = SonicatorCommand.Request()
        request.timer.value  = timer
        request.timer.unit = "minute"
        request.temperature = temperature
        request.job_id = job_id
        future = self.sonicator_control_srv.call_async(request)
        await future
        response:SonicatorCommand.Response = future.result()
        if not response.success:
            self.logger(f"Failed to sonicate with reason: {response.status.reason}")
            if self.should_raise:
                raise Exception(f"Sonicate Failed: {response.status.reason}")
        return
    
    async def heating(self, temperature: QuantityValue, job_id=""):
        if not self.sonicator_control_srv.service_is_ready():
            raise Exception("Sonicator Service is not ready")
        request = SonicatorCommand.Request()
        request.timer.value  = 1.0 # Will take default time for heating
        request.timer.unit = "second"
        request.temperature = temperature
        request.job_id = job_id
        future = self.heating_control_srv.call_async(request)
        await future
        response:SonicatorCommand.Response = future.result()
        if not response.success:
            self.logger(f"Failed to sonicate with reason: {response.status.reason}")
            if self.should_raise:
                raise Exception(f"Sonicate Failed: {response.status.reason}")
        return
    
    def prep(self):
        raise Exception("Can't move the Magnetic Stirrer")

    def pick(self):
        raise Exception("Can't move the Magnetic Stirrer")

    def place(self, *_):
        raise Exception("Can't move the Magnetic Stirrer")
