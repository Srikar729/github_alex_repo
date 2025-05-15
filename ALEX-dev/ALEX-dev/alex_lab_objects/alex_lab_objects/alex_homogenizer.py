from geometry_msgs.msg import Point
from alex_interfaces.msg import QuantityValue
from alex_interfaces.srv import HomogenizerCommand
from alex_lab_objects.alex_object import ALEXObject

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class Homogenizer(ALEXObject):
    def __init__(self, position: Point, homogenizer_control_srv: Client, *, should_raise: bool = True, logger:RcutilsLogger | None = None):
        ALEXObject.__init__(self, position)
        self.logger = lambda a: logger.info(f"[{self.__class__.__name__}]: "+f"{a}") if logger else print(f"[{self.__class__.__name__}]: "+f"{a}")
        self.homogenizer_control_srv = homogenizer_control_srv
        self.should_raise = should_raise

    async def mix(self, param: int, quantity: QuantityValue, job_id=""):
        if not self.homogenizer_control_srv.service_is_ready():
            raise Exception("Homogenizer Service is not ready")
        request = HomogenizerCommand.Request()
        request.rpm    = param
        request.timer  = quantity
        request.job_id = job_id
        future = self.homogenizer_control_srv.call_async(request)
        await future
        response:HomogenizerCommand.Response = future.result()
        if not response.success:
            self.logger(f"Failed to Homogenize with reason: {response.status.reason}")
            if self.should_raise:
                raise Exception(f"Homogenizing Failed: {response.status.reason}")
            return
        return
    
    def prep(self):
        raise Exception("Can't move the Homogenizer")

    def pick(self):
        raise Exception("Can't move the Homogenizer")

    def place(self, *_):
        raise Exception("Can't move the Homogenizer")
