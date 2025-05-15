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
# ROS Interface Import
from alex_interfaces.msg import SerialMessage, LiquidVolumeStatus
from alex_interfaces.srv import LiquidLevelCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.external_constants import SerialBand
# Python Import
import math
from collections import deque

class LiquidLevel(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        
        self.parameters()
        self.set_publishers()
        self.set_subscribers()
        self.set_services()

        self.initialize()

        self.logger.info(f"{node_name} started running")
        
    def parameters(self,):
        # declare ros 2 parameters
        parameters = dict(
            container_type = "cuboid",
            max_height     = 15.0,
            length         = 8.0,
            width          = 8.0,
            radius         = 0.0,
            sensor_offset  = 2.0,
            critical_ml    = 500,
            deque_maxlen   = 10,
        )
        self.declare_parameters("", parameters=parameters.items())
        
        # get ros 2 parameters
        self.container_type:str  = self.get_parameter('container_type').value
        
        self.max_height:float    = self.get_parameter('max_height').value
        self.length:float        = self.get_parameter('length').value
        self.width:float         = self.get_parameter('width').value
        
        self.radius:float        = self.get_parameter('radius').value
        
        self.sensor_offset:float = self.get_parameter('sensor_offset').value
        self.critical_ml:int     = self.get_parameter('critical_ml').value
        self.deque_maxlen:int    = self.get_parameter('deque_maxlen').value

    def set_publishers(self,):
        self.pub_liquid_volume  = self.create_publisher(LiquidVolumeStatus, 'liquid_volume', 10)

    def set_subscribers(self,):
        self.sub_from_serial = self.create_subscription(SerialMessage, 'from_serial', self.handle_serial_data, 10)
    
    def set_services(self,):
        self.create_service(LiquidLevelCommand, "start_liquid_level_reading", self.handle_liquid_level_reading )

    def initialize(self,):
        self.liquid_level_deque:deque[int] = deque(maxlen=self.deque_maxlen)
        self.average_volume = 0.0

        self.volume_publish_timer = self.create_timer(60.0, self.publish_liquid_level)

    def calculate_volume(self,empty_liquid_level:int|float):
        if self.container_type == "cylinder":
            calculate_volume = self.calculate_cylinder_volume
        elif self.container_type == "cuboid":
            calculate_volume = self.calculate_cuboid_volume
        else:
            self.logger.warning(f"Got invalid {self.container_type}. Default to cuboid")
            calculate_volume = self.calculate_cuboid_volume
        
        liquid_height = self.max_height - self.sensor_offset - empty_liquid_level
        liquid_height = max(liquid_height, 0)
        volume = calculate_volume(liquid_height)
        return volume
    
    def calculate_cylinder_volume(self, liquid_height:float):
        volume = math.pi * self.radius**2 * liquid_height
        return volume
    
    def calculate_cuboid_volume(self, liquid_height:float):
        volume = self.length * self.width * liquid_height
        return volume
    
    def get_calculate_volume(self, mode = LiquidLevelCommand.Request.COMMAND_LATEST):
        if mode == LiquidLevelCommand.Request.COMMAND_AVERAGE:
            empty_area_level = sum(self.liquid_level_deque)/len(self.liquid_level_deque)
        else:
            empty_area_level = self.liquid_level_deque[-1]
        volume = self.calculate_volume(empty_area_level)
        return volume

    def ros_publisher_decorator(func):
        def wrapper(self, request:LiquidLevelCommand.Request , response:LiquidLevelCommand.Response):
            result:LiquidLevelCommand.Response = func(self, request, response)
            result.status.job_id = request.job_id
            self.pub_liquid_volume.publish(result.status)
            return result
        return wrapper
    
    @ros_publisher_decorator
    def handle_liquid_level_reading(self, request: LiquidLevelCommand.Request, response: LiquidLevelCommand.Response):
        response.status.value.unit = "ml"
        if request.command not in [LiquidLevelCommand.Request.COMMAND_LATEST, LiquidLevelCommand.Request.COMMAND_AVERAGE]:
            response.success = False
            response.status.status = LiquidVolumeStatus.STATUS_ERROR
            response.status.reason = "Invalid command given"
            return response
        if not self.liquid_level_deque:
            response.success = False
            response.status.status = LiquidVolumeStatus.STATUS_ERROR
            response.status.reason = "Did not get any liquid_level yet"
            return response
        response.success = True
        response.status.status = LiquidVolumeStatus.STATUS_COMPLETED
        response.status.value.value = self.get_calculate_volume(request.command)
        return response
    
    def publish_liquid_level(self,):
        if not self.liquid_level_deque:
            self.logger.warning("Did not get liquid_level from serial")
            return
        average_volume = self.get_calculate_volume(LiquidLevelCommand.Request.COMMAND_AVERAGE)
        if average_volume == self.average_volume:
            self.logger.debug("Found same average liquid volume")
            return
        self.average_volume = average_volume
        msg = LiquidVolumeStatus(status=LiquidVolumeStatus.STATUS_COMPLETED)
        msg.value.unit = "ml"
        msg.job_id
        if self.average_volume <= self.critical_ml:
            msg.status = LiquidVolumeStatus.STATUS_ERROR
            msg.reason = f"Liquid level below critical level: {average_volume}/{self.critical_ml}"
        msg.value.value = average_volume
        self.pub_liquid_volume.publish(msg)

    def handle_serial_data(self, data:SerialMessage):
        if not data.data.startswith(SerialBand.LIQUID_LEVEL.value): return

        value = data.data.removeprefix(SerialBand.LIQUID_LEVEL.value)
        if not value.isdigit():
            self.logger.error(f"Got invalid data for liquid level {value}")
            return
        self.logger.debug(value)
        self.liquid_level_deque.append(int(value))

def main(args=None):
    rclpy.init(args=args)
    node = LiquidLevel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
