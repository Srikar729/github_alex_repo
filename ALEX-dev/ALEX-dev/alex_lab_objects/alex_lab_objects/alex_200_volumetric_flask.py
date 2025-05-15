from geometry_msgs.msg import Point, Vector3

from alex_lab_objects.alex_container import Container
from alex_lab_objects.alex_constans import ContainerType
from alex_interfaces.srv import ArmCommand

from time import sleep

from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger

class VolumetricFlask200(Container):
    def __init__(self, position: Point, cobot_control_srv:Client, logger:RcutilsLogger=None):
        type = ContainerType.ML_200_VOLUMETRIC_FLASK
        super().__init__(type, position, cobot_control_srv, logger)
        
        self.robot_control_srv = cobot_control_srv
    
    async def pour(self):
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
