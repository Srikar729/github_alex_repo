from geometry_msgs.msg import Point, Vector3

from alex_lab_objects.alex_container import Container
from alex_lab_objects.alex_constans import ContainerType
from alex_interfaces.srv import ArmCommand

from time import sleep

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class WeightBoat(Container):
    def __init__(self, position: Point, cobot_control_srv:Client, logger:RcutilsLogger=None):
        type = ContainerType.WEIGHTBOAT
        super().__init__(type, position, cobot_control_srv, logger)
        
        self.robot_control_srv = cobot_control_srv

    async def pour(self):
        raise NotImplementedError("Pour is not allowed for weight boat")
    
    async def mount(self,):
        raise NotImplementedError("Should add these feature")

    async def discard(self,):
        raise NotImplementedError("Should add these feature")

    async def pick(self):
        srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
        pickup_point = self.point_offset_origin(self.position.x, self.position.y, offset=-30)
        srv_request.position = Point(x=pickup_point[0], y=pickup_point[1], z=self.position.z)
        self.logger(f'Got pick up pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        future = self.robot_control_srv.call_async(srv_request)
        await future


        self.logger(f'Grasp the container')
        future = self.robot_control_srv.call_async(ArmCommand.Request(command=ArmCommand.Request.PICK))
        await future

        
        self.logger(f'Lift the container')
        srv_request.position.z += self._lift_height
        future = self.robot_control_srv.call_async(srv_request)
        await future

        
        self.logger('Taking container back')
        post_pickup_point = self.point_offset_origin(self.position.x, self.position.y, offset=150)
        srv_request.command = ArmCommand.Request.MOVE
        srv_request.position = Point(x=post_pickup_point[0], y=post_pickup_point[1], z=self.position.z+self._lift_height)
        future = self.robot_control_srv.call_async(srv_request)
        await future
    
    async def place(self, target_location: Point):
        #TODO: Can avoid target location
        srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
        post_destination_point = self.point_offset_origin(target_location.x, target_location.y, offset=-30)
        srv_request.position = Point(x=post_destination_point[0], y=post_destination_point[1], z=target_location.z)
        self.logger(f'Place the container down')
        future = self.robot_control_srv.call_async(srv_request)
        await future


        self.logger(f'Release the container')
        future = self.robot_control_srv.call_async(
            ArmCommand.Request(
                command=ArmCommand.Request.PICK, 
                picking_group=ArmCommand.Request.GROUP3)
            )
        await future


        post_destination_point = self.point_offset_origin(target_location.x, target_location.y, offset=50)
        srv_request.position = Point(x=post_destination_point[0], y=post_destination_point[1], z=target_location.z)  
        self.logger(f'Got post destimation pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        future = self.robot_control_srv.call_async(srv_request)
        await future

        
