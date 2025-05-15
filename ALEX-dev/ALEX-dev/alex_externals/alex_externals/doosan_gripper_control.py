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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
# ROS Interface Import
from dsr_msgs2.srv import SetToolDigitalOutput
from alex_interfaces.srv import GripperCommand
# Python Import
import asyncio
from time import sleep
# Utilities Import
from alex_utilities.common_utilities import change_case

class DoosanGripperControl(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.parameters()

        self.create_doosan_service_clients(self.gripper_namespace)

        self.doosan_arm_control_service = self.create_service(
            GripperCommand,
            "gripper_control",
            self.gripper_control,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.logger.info(f"{node_name} started running")
    
    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('client_max_attempt_count', value=5       , descriptor=ParameterDescriptor(description="Max attempt wait for connecting to doosan client"))
        self.declare_parameter('gripper_delay_time',       value=2       , descriptor=ParameterDescriptor(description="End Effector delay time"))
        self.declare_parameter('gripper_namespace',        value="/dsr01", descriptor=ParameterDescriptor(description="Namespace of the gripper"))
        
        # get ros 2 parameters
        self.max_attempt_count:int     = self.get_parameter('client_max_attempt_count').value
        self.gripper_delay_time:int    = self.get_parameter('gripper_delay_time').value
        self.gripper_namespace:str     = self.get_parameter('gripper_namespace').value
            
    def create_doosan_service_clients(self, namespace:str):
        attempt_count = 0
        # Waiting for tool_output_client
        self.tool_output_client = self.create_client(SetToolDigitalOutput, f'{namespace}/io/set_tool_digital_output')
        while not self.tool_output_client.wait_for_service(timeout_sec=1.0) :
            attempt_count += 1
            self.logger.info(f'tool_output_client Service not available, trying again... attempt {attempt_count}/{self.max_attempt_count}')
            if attempt_count==self.max_attempt_count:
                raise Exception("tool_output_client services not available")
        self.logger.info(f'tool_output_client Service found.')
        attempt_count = 0
        
    async def gripper_hold(self, group:int=GripperCommand.Request.GROUP4):
        match group:
            case GripperCommand.Request.GROUP2:
                gripper_goals = [
                    SetToolDigitalOutput.Request(index = 1, value = 1),
                    SetToolDigitalOutput.Request(index = 2, value = 0),
                ]
            case GripperCommand.Request.GROUP3:
                gripper_goals = [
                    SetToolDigitalOutput.Request(index = 1, value = 0),
                    SetToolDigitalOutput.Request(index = 2, value = 1),
                ]
            case GripperCommand.Request.GROUP4:
                gripper_goals = [
                    SetToolDigitalOutput.Request(index = 1, value = 1),
                    SetToolDigitalOutput.Request(index = 2, value = 1),
                ]
            case _:
                self.logger.warning("Got Invalid gripper group. Default to Group 4")
                gripper_goals = [
                    SetToolDigitalOutput.Request(index = 1, value = 1),
                    SetToolDigitalOutput.Request(index = 2, value = 1),
                ]

        results = [await self.tool_output_client.call_async(goal) for goal in gripper_goals]
        value = all((result.success) for result in results)
        sleep(self.gripper_delay_time)
        self.logger.info(f"Gripper Hold: {value=}")
    
    async def gripper_release(self):
        # Group 1
        gripper_goals = [
            SetToolDigitalOutput.Request(index = 1, value = 0),
            SetToolDigitalOutput.Request(index = 2, value = 0),
        ]
        results = [await self.tool_output_client.call_async(goal) for goal in gripper_goals]
        value = all((result.success for result in results))
        sleep(self.gripper_delay_time)
        self.logger.info(f"Gripper Release: {value=}")
            
    async def gripper_control(self, request: GripperCommand.Request, response: GripperCommand.Response):
        match request.command:
            case GripperCommand.Request.PICK:
                await self.gripper_hold(request.picking_group)
            case GripperCommand.Request.DROP:
                await self.gripper_release()
            case _:
                self.logger.warning("Got Invalid gripper command")
                response.reason = "Got invalid command"
                return response
        #TODO: need to add better response from each switch function
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoosanGripperControl()
    
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(rclpy.spin(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
