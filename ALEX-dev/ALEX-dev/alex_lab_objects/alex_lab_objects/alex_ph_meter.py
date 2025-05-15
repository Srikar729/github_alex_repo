from geometry_msgs.msg import Point
from alex_interfaces.srv import MeasurePhCommand
from alex_lab_objects.alex_object import ALEXObject
from alex_lab_objects.alex_container import Container

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class PHMeter(ALEXObject):
    
    def __init__(self, position: Point, ph_control_srv: Client, logger:RcutilsLogger | None = None, *, should_raise:bool = True):
        ALEXObject.__init__(self, position)
        self.logger = lambda a: logger.info(f"[{self.__class__.__name__}]: "+f"{a}") if logger else print(f"[{self.__class__.__name__}]: "+f"{a}")
        self.ph_control_srv = ph_control_srv
        self.should_raise = should_raise

    async def measure(self, target: Container, job_id=""):
        request = MeasurePhCommand.Request()
        request.delay = 5
        request.distance = 32 #TODO: Should be calculated from target height
        request.mode = MeasurePhCommand.Request.LASTVALUE
        request.job_id=job_id
        future = self.ph_control_srv.call_async(request)
        await future
        response:MeasurePhCommand.Response = future.result()
        if not response.success:
            self.logger(f"Failed to measure with reason: {response.reason}")
            if self.should_raise:
                raise Exception(f"Measure Failed: {response.reason}")
            return -1.0
        return response.value
