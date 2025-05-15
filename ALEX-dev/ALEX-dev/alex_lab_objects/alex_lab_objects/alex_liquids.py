from geometry_msgs.msg import Point
from alex_interfaces.srv import LiquidDoserCommand
from alex_lab_objects.alex_object import ALEXObject

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class Liquid(ALEXObject):
    
    def __init__(self, position: Point, liquid_dosing_srv: Client, logger:RcutilsLogger | None = None, *, should_raise:bool = True):
        ALEXObject.__init__(self, position)

        self.logger = logger
        if not self.logger:
            self.logger = lambda a: print(f"[{self.__class__.__name__}]: {a}")
            self.logger.info = lambda a: print(f"[{self.__class__.__name__}] [INFO]: {a}")
            self.logger.error = lambda a: print(f"[{self.__class__.__name__}] [ERROR]: {a}")
            self.logger.fatal = lambda a: print(f"[{self.__class__.__name__}] [FATAL]: {a}")

        self.liquid_dosing_srv = liquid_dosing_srv
        self.should_raise = should_raise

    async def pour(self, value:float, job_id:str = ""):
        self.logger.info(f"Pouring {value} ml of liquid with job ID: {job_id}")
        request = LiquidDoserCommand.Request()
        request.command = LiquidDoserCommand.Request.COMMAND_DISPENSE
        request.value = value
        request.job_id = job_id
        future = self.liquid_dosing_srv.call_async(request)
        await future
        response:LiquidDoserCommand.Response = future.result()
        if not response.success:
            self.logger.error(f"Failed to Pour with reason: {response.status.reason}")
            if self.should_raise:
                raise Exception(f"Pour Failed: {response.status.reason}")
        return response.status.value.value
