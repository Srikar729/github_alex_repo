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
from alex_interfaces.srv import RobotCommand
from dsr_msgs2.srv import MoveJoint, MoveJointx, GetCurrentPosj, GetCurrentPosx
# Python Import
import math
import asyncio
from time import sleep
# Utilities Import
from alex_utilities.common_utilities import change_case

class DoosanArmControl(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.parameters()
        self.end_effector_joint = 6

        self.create_doosan_service_clients(self.robot_namespace)

        self.doosan_arm_control_service = self.create_service(
            RobotCommand,
            "move_robot",
            self.move_robot,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.logger.info(f"{node_name} started running")
    
    @staticmethod
    def get_orientation(x:float, y:float):
        """
        finds the A(Yaw) with respect to the robot origin. 
        """
        # https://math.stackexchange.com/questions/1201337/finding-the-angle-between-two-points
        a = math.degrees(math.atan2(y, x))
        if a < 0:
            # The robot only takes positive value.
            return a + 180
        return a

    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('robot_namespace',          value="/dsr01", descriptor=ParameterDescriptor(description="Namespace of the robot"))
        self.declare_parameter('client_max_attempt_count', value=5    , descriptor=ParameterDescriptor(description="Max attempt wait for connecting to doosan client"))
        self.declare_parameter('doosan_arm_delay',         value=0.5  , descriptor=ParameterDescriptor(description="Doosan arm operation delay"))
        self.declare_parameter('default_gripper_hold_angle',value=-90.0, descriptor=ParameterDescriptor(description="Maximum Doosan accleration for arm movement"))
        self.declare_parameter('default_velocity',         value=100.5, descriptor=ParameterDescriptor(description="Doosan velocity for arm movement(degree/sec)"))
        self.declare_parameter('default_accleration',      value=60.0 , descriptor=ParameterDescriptor(description="Doosan accleration for arm movement"))
        self.declare_parameter('max_velocity',             value=100.0, descriptor=ParameterDescriptor(description="Maximum Doosan velocity for arm movement(degree/sec)"))
        self.declare_parameter('max_accleration',          value=60.0 , descriptor=ParameterDescriptor(description="Maximum Doosan accleration for arm movement"))
        self.declare_parameter('robot_pack_position',      value=[0.0]*6 , descriptor=ParameterDescriptor(description="Position for robot joints to pack"))
        self.declare_parameter('robot_unpack_position',    value=[0.0]*6 , descriptor=ParameterDescriptor(description="Position for robot joints to unpack"))

        # get ros 2 parameters
        self.robot_namespace:str       = self.get_parameter('robot_namespace').value
        self.max_attempt_count:int     = self.get_parameter('client_max_attempt_count').value
        self.default_gripper_hold_angle:int= self.get_parameter('default_gripper_hold_angle').value
        self.doosan_arm_delay:int      = self.get_parameter('doosan_arm_delay').value
        self.default_velocity:int      = self.get_parameter('default_velocity').value
        self.default_accleration:int   = self.get_parameter('default_accleration').value
        self.max_velocity:int          = self.get_parameter('max_velocity').value
        self.max_accleration:int       = self.get_parameter('max_accleration').value
        self.robot_pack_position:int   = self.get_parameter('robot_pack_position').value
        self.robot_unpack_position:int = self.get_parameter('robot_unpack_position').value
        
    
    def create_doosan_service_clients(self, namespace:str):
        attempt_count = 0
        # Waiting for move_joint_client
        self.move_joint_client = self.create_client(MoveJoint, f'{namespace}/motion/move_joint')
        while not self.move_joint_client.wait_for_service(timeout_sec=1.0) :
            attempt_count += 1
            self.logger.info(f'move_joint_client Service not available, tring again... attempt {attempt_count}/{self.max_attempt_count}')
            if attempt_count==self.max_attempt_count:
                raise Exception("move_joint_client services not available")
        self.logger.info(f'move_joint_client Service found.')
        # Waiting for move_jointx_client
        self.move_jointx_client = self.create_client(MoveJointx, f'{namespace}/motion/move_jointx')
        while not self.move_jointx_client.wait_for_service(timeout_sec=1.0) :
            attempt_count += 1
            self.logger.info(f'move_jointx_client Service not available, tring again... attempt {attempt_count}/{self.max_attempt_count}')
            if attempt_count==self.max_attempt_count:
                raise Exception("move_jointx_client services not available")
        self.logger.info(f'move_jointx_client Service found.')
        attempt_count = 0
        # Waiting for get_posj_client
        self.get_posj_client = self.create_client(GetCurrentPosj, f'{namespace}/aux_control/get_current_posj')
        while not self.get_posj_client.wait_for_service(timeout_sec=1.0) :
            attempt_count += 1
            self.logger.info(f'get_posj_client Service not available, tring again... attempt {attempt_count}/{self.max_attempt_count}')
            if attempt_count==self.max_attempt_count:
                raise Exception("get_posj_client services not available")
        self.logger.info(f'get_posj_client Service found.')
        attempt_count = 0
        # Waiting for get_posx_client
        self.get_posx_client = self.create_client(GetCurrentPosx, f'{namespace}/aux_control/get_current_posx')
        while not self.get_posx_client.wait_for_service(timeout_sec=1.0) :
            attempt_count += 1
            self.logger.info(f'get_posx_client Service not available, tring again... attempt {attempt_count}/{self.max_attempt_count}')
            if attempt_count==self.max_attempt_count:
                raise Exception("get_posx_client services not available")
        self.logger.info(f'get_posx_client Service found.')
        attempt_count = 0

    async def move_to_goal(self, x:float, y:float, z:float):
        negation_factor = 1 if y > 0 else -1
        
        updated_a = self.get_orientation(x, y)
        updated_b, updated_c = 90.0 * negation_factor, 90.0 * negation_factor

        move_jointx_goal = MoveJointx.Request()
        move_jointx_goal.pos = [ x, y, z, updated_a, updated_b, updated_c ]
        move_jointx_goal.vel = self.default_velocity
        move_jointx_goal.acc = self.default_accleration
        move_jointx_goal.sol = 2

        future = self.move_jointx_client.call_async(move_jointx_goal)
        await future
        result:MoveJointx.Response = future.result()
        value = result.success
        self.logger.info(f"Reached Goal: {x=} {y=} {z=} {value=}")
    
    async def gripper_turn(self, angle:float, speed_scale_factor: float=0.5, acceleration_scale_factor: float=1):
        pos_future = self.get_posj_client.call_async(GetCurrentPosj.Request())
        await pos_future
        pos_result:GetCurrentPosj.Response = pos_future.result()
        if not pos_result.success:
            raise Exception("Did not get the robot's current posj")
        current_posj = pos_result.pos

        goal_posj = current_posj
        goal_posj[self.end_effector_joint-1] = angle

        speed        = self.default_velocity * speed_scale_factor
        acceleration = self.default_accleration * acceleration_scale_factor

        # FIXME: Wrong way to do it
        if speed > self.max_velocity:
            self.logger.warning("speed above maximum")
        if acceleration > self.max_accleration:
            self.logger.warning("Acceleration above maximum")
        
        movej_goal = MoveJoint.Request()
        movej_goal.pos = goal_posj
        movej_goal.vel = min(speed, self.max_velocity)
        movej_goal.acc = min(acceleration, self.max_accleration)

        future = self.move_joint_client.call_async(movej_goal)
        await future
        result:MoveJoint.Response = future.result()
        value = result.success

        self.logger.info(f"Gripper Turn done: {angle} {value=}")
    
    async def relative_movement(self, x:float = 0, y:float = 0, z:float = 0):

        pos_future = self.get_posx_client.call_async(GetCurrentPosx.Request(ref=0))
        await pos_future
        pos_result:GetCurrentPosx.Response = pos_future.result()
        if not pos_result.success:
            raise Exception("Did not get the robot's current posj")
        *current_posx, solution_space = pos_result.task_pos_info[0].data
        self.logger.debug(f"current_posx {current_posx}")
        
        goal_posx = current_posx
        goal_posx[0] += x
        goal_posx[1] += y
        goal_posx[2] += z

        move_jointx_goal = MoveJointx.Request()
        move_jointx_goal.pos = goal_posx
        move_jointx_goal.vel = self.default_velocity
        move_jointx_goal.acc = self.default_accleration
        move_jointx_goal.sol = 2

        future = self.move_jointx_client.call_async(move_jointx_goal)
        await future
        result:MoveJointx.Response = future.result()
        value = result.success

        self.logger.info(f"Minor adjustments done: {x=} {y=} {z=} {value=}")
    
    async def stir_inplace(self):

        await self.gripper_turn(self.default_gripper_hold_angle + 30, speed_scale_factor=5, acceleration_scale_factor= 4); sleep(self.doosan_arm_delay)
        await self.gripper_turn(self.default_gripper_hold_angle - 30, speed_scale_factor=5, acceleration_scale_factor= 4); sleep(self.doosan_arm_delay)
        
        await self.gripper_turn(self.default_gripper_hold_angle + 30, speed_scale_factor=5, acceleration_scale_factor= 4); sleep(self.doosan_arm_delay)
        await self.gripper_turn(self.default_gripper_hold_angle - 30, speed_scale_factor=5, acceleration_scale_factor= 4); sleep(self.doosan_arm_delay)
        
        await self.gripper_turn(self.default_gripper_hold_angle)
        self.logger.info(f"Stirring done")
    
    async def move_to_pack_unpack(self, mode="unpack"):
        movej_goal = MoveJoint.Request()
        if mode == "pack":
            movej_goal.pos = self.robot_pack_position
        else: 
            movej_goal.pos = self.robot_unpack_position
        movej_goal.vel = self.default_velocity
        movej_goal.acc = self.default_accleration

        future = self.move_joint_client.call_async(movej_goal)
        await future
        result:MoveJoint.Response = future.result()
        value = result.success
        return value

    async def move_robot(self, request: RobotCommand.Request, response: RobotCommand.Response):
        match request.command:
            case RobotCommand.Request.MOVE:
                await self.move_to_goal(request.position.x, request.position.y, request.position.z)
            case RobotCommand.Request.TURN:
                await self.gripper_turn(request.angle.z, 0.2)
            case RobotCommand.Request.RELATIVE_MOVEMENT:
                await self.relative_movement(request.difference.x, request.difference.y, request.difference.z)
            case RobotCommand.Request.STIR:
                await self.stir_inplace()
            case RobotCommand.Request.PACK | RobotCommand.Request.UNPACK:
                await self.move_to_pack_unpack(mode=request.command)
            case _:
                self.logger.warning("Got invalid command")
                response.reason = "Got invalid command"
                return response
        #BUG: For some reason arm crash with error index: 7056(OPERATION_SAFETY_FUNCTION_SOS_VIOLATION). Sleep somehow helps prevent the crash
        sleep(self.doosan_arm_delay)
        #TODO: need to add better response from each switch function
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoosanArmControl()
    
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(rclpy.spin(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
