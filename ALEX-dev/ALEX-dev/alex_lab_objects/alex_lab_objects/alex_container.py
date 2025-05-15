# import rospy
from geometry_msgs.msg import Point, Vector3

from alex_lab_objects.alex_object import ALEXObject
from alex_lab_objects.alex_constans import ContainerType, CONTAINER_DIMENTIONS, LIFT_HEIGHT
from alex_interfaces.srv import ArmCommand

from time import sleep

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class Container(ALEXObject):
    
    def __init__(self, type: ContainerType, position: Point, cobot_control_srv: Client, logger:RcutilsLogger | None = None):
        ALEXObject.__init__(self, position)
        self.logger = lambda a: logger.info(f"[{self.__class__.__name__}]: "+f"{a}") if logger else print(f"[{self.__class__.__name__}]: "+f"{a}")
        self._lift_height:int = LIFT_HEIGHT
        self.type = type
        self.size = CONTAINER_DIMENTIONS[type]
        self.robot_control_srv = cobot_control_srv
        
    async def prep(self):
        self.logger(f'Open gripper fully for prep')
        future = self.robot_control_srv.call_async(ArmCommand.Request(command=ArmCommand.Request.DROP))
        await future
        
        # Safety movement
        srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
        srv_request.position = Point(x=-432.50, y=10.51, z=100.0)
        self.logger(f'Got savety destimation pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        future = self.robot_control_srv.call_async(srv_request)
        await future


        srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
        pre_pickup_point = self.point_offset_origin(self.position.x, self.position.y, offset=50)
        srv_request.position = Point(x=pre_pickup_point[0], y=pre_pickup_point[1], z=self.position.z)
        self.logger(f'moving container to prep position for container: x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        self.logger(srv_request.position)
        future = self.robot_control_srv.call_async(srv_request)
        await future

        
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
        post_pickup_point = self.point_offset_origin(self.position.x, self.position.y, offset=50)
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
        future = self.robot_control_srv.call_async(ArmCommand.Request(command=ArmCommand.Request.DROP))
        await future


        post_destination_point = self.point_offset_origin(target_location.x, target_location.y, offset=50)
        srv_request.position = Point(x=post_destination_point[0], y=post_destination_point[1], z=target_location.z)  
        self.logger(f'Got post destimation pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        future = self.robot_control_srv.call_async(srv_request)
        await future

        
    async def move(self, target_location: Point, pre_destination=True):
        # Safety movement
        srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
        srv_request.position = Point(x=-432.50, y=10.51, z=self.position.z+self._lift_height)
        self.logger(f'Got savety destimation pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        future = self.robot_control_srv.call_async(srv_request)
        await future


        if pre_destination:
            srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
            pre_destination_point = self.point_offset_origin(target_location.x, target_location.y, offset=150)
            srv_request.position = Point(x=pre_destination_point[0], y=pre_destination_point[1], z=target_location.z+self._lift_height)
            self.logger(f'Got pre destimation pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
            future = self.robot_control_srv.call_async(srv_request)
            await future
        
        # Actual Destination
        srv_request = ArmCommand.Request(command=ArmCommand.Request.MOVE)
        post_destination_point = self.point_offset_origin(target_location.x, target_location.y, offset=-30)
        srv_request.position = Point(x=post_destination_point[0], y=post_destination_point[1], z=target_location.z+self._lift_height)
        self.logger(f'Got destimation pose : x={srv_request.position.x} y={srv_request.position.y} z={srv_request.position.z}')
        future = self.robot_control_srv.call_async(srv_request)
        await future


        self.position = target_location
    
    async def pour(self):
        if self.type == ContainerType.ML_200_FLASK:

            # Move closer
            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=20.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = -60
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = -30
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=30.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 0
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 30
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=30.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 60
            future = self.robot_control_srv.call_async(srv_request)
            await future

            # Final drops pouring
            sleep(2)
            # Bring it back up

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 0
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=-60.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future

            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = -90
            future = self.robot_control_srv.call_async(srv_request)
            await future
            return
        elif self.type == ContainerType.ML_200_VOLUMETRIC_FLASK:
            # Move a bit far
            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=30.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Turn the flask
            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = -30.0
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Turn the flask
            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 0.0
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Move closer
            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=15.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Turn the flask
            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 30.0
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Move closer
            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=55.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Turn the flask
            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 60.0
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Final drops pouring
            sleep(2)
            # Bring it back up
            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = 0.0
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Go back a bit to be safe
            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=-90.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Turn back
            srv_request = ArmCommand.Request(command=ArmCommand.Request.TURN)
            srv_request.angle.z = -90.0
            future = self.robot_control_srv.call_async(srv_request)
            await future
            # Go back a little further
            srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
            srv_request.difference = Vector3(x=-60.0, y=0.0, z=0.0)
            future = self.robot_control_srv.call_async(srv_request)
            await future
            
    async def stir(self):
        # Move closer
        srv_request = ArmCommand.Request(command=ArmCommand.Request.RELATIVE_MOVEMENT)
        srv_request.difference = Vector3(x=0.0, y=0.0, z=90.0)
        future = self.robot_control_srv.call_async(srv_request)
        await future

        # Start stiring
        srv_request = ArmCommand.Request(command=ArmCommand.Request.STIR)
        future = self.robot_control_srv.call_async(srv_request)
        await future
