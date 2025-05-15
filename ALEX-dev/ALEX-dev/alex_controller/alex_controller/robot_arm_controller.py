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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# ROS Interface Import
from alex_interfaces.srv import GripperCommand, RobotCommand, ArmCommand
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
pass

class RobotArmController(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.initialize()
        self.parameters()
        self.set_services()
        self.create_service_client()

        self.logger.info(f"{node_name} started running")
    
    def initialize(self,):
        pass
    
    def parameters(self,):
        pass

    def create_service_client(self,):
        self.move_robot_srv      = self.create_client(RobotCommand, "move_robot")
        self.gripper_control_srv = self.create_client(GripperCommand, "gripper_control")
                         
    def set_services(self,):
        self.create_service(ArmCommand, "arm_control", self.handle_arm_control, callback_group=MutuallyExclusiveCallbackGroup())

    async def handle_arm_control(self, request:ArmCommand.Request, response:ArmCommand.Response):
        self.logger.info(f"Received arm control request, Command: {request.command}, Pos: {request.position}, diff: {request.difference}, picking_group: {request.picking_group}")

        # Check if the service is available
        if request.command in [ArmCommand.Request.DROP, ArmCommand.Request.PICK]:
            service = self.gripper_control_srv
            error_msg = "Gripper control service is not available"
        else:
            service = self.move_robot_srv
            error_msg = "Move robot service is not available"

        if not service.service_is_ready():
            response.reason = error_msg
            self.logger.error(error_msg)
            return response
        
        match request.command:
            case ArmCommand.Request.MOVE:
                self.logger.info(f"Moving arm to position {request.position}")
                command_request = RobotCommand.Request()
                command_request.command  = RobotCommand.Request.MOVE
                command_request.position = request.position
                command_response = self.move_robot_srv.call_async(command_request)
            case ArmCommand.Request.RELATIVE_MOVEMENT:
                self.logger.info(f"Relative Movement: {request.difference}")
                command_request = RobotCommand.Request()
                command_request.command  = RobotCommand.Request.RELATIVE_MOVEMENT
                command_request.difference = request.difference
                command_response = self.move_robot_srv.call_async(command_request)
            case ArmCommand.Request.TURN:
                self.logger.info(f"Turning arm to angle: {request.angle}")
                command_request = RobotCommand.Request()
                command_request.command  = RobotCommand.Request.RELATIVE_MOVEMENT
                command_request.angle = request.angle
                command_response = self.move_robot_srv.call_async(command_request)
            case ArmCommand.Request.PICK:
                self.logger.info(f"Gripper Pick group: {request.picking_group}")
                command_request = GripperCommand.Request()
                command_request.command  = GripperCommand.Request.PICK
                command_request.picking_group = request.picking_group
                command_response = self.gripper_control_srv.call_async(command_request)
            case ArmCommand.Request.DROP:
                self.logger.info(f"Gripper Drop")
                command_request = GripperCommand.Request()
                command_request.command  = GripperCommand.Request.DROP
                command_response = self.gripper_control_srv.call_async(command_request)
            case ArmCommand.Request.STIR:
                self.logger.info(f"Stirring")
                command_request = RobotCommand.Request()
                command_request.command  = RobotCommand.Request.STIR
                command_response = self.move_robot_srv.call_async(command_request)
            case ArmCommand.Request.PACK:
                self.logger.info(f"Packing")
                command_request = RobotCommand.Request()
                command_request.command  = RobotCommand.Request.PACK
                command_response = self.move_robot_srv.call_async(command_request)
            case ArmCommand.Request.UNPACK:
                self.logger.info(f"Unpacking")
                command_request = RobotCommand.Request()
                command_request.command  = RobotCommand.Request.UNPACK
                command_response = self.move_robot_srv.call_async(command_request)
            case _:
                response.reason = "Invalid command"
                self.logger.error(response.reason)
                return response
        await command_response
        response_result:ArmCommand.Response = command_response.result()
        response.success = response_result.success
        if not response.success:
            response.reason = f"Failed with reason: {response_result.reason}"
        self.logger.info(f"Arm control response: success={response.success}, reason={response.reason}")        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmController()
    executor = MultiThreadedExecutor(num_threads=2) # 1 for spin, 1 for async service
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
