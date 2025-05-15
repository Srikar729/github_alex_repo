"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 

"""
# Python Import
import math
import asyncio
# ROS Import
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from dsr_msgs2.srv import MoveJointx
from alex_interfaces.srv import ArmCommand, ObjectPoseDetection
# Utilities Import
from alex_utilities.common_utilities import change_case

class FollowObjects(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()
        self.PREP_RANGE = 450.0
        self.create_service_client()
        self.demo_task = self.create_timer(5.0, self.run_demo_loop)
        self.logger.info(f"{node_name} started running")          
  
    def create_service_client(self):
        '''
        Temp solution to get current pose
        Will be added to doosan_arm_control
        '''
        self.move_jointx_client = self.create_client(MoveJointx, '/dsr01/motion/move_jointx', callback_group=ReentrantCallbackGroup())
        self.doosan_arm_control_srv = self.create_client(ArmCommand, "doosan_arm_control", callback_group=ReentrantCallbackGroup())
        self.object_detector = self.create_client(ObjectPoseDetection, "object_detector", callback_group=ReentrantCallbackGroup())
        
    async def move_to_relative_goal(self, x: float, y: float, z: float) -> Future:
        self.logger.info(f'moving to relative goal sync {x=}, {y=}, {z=}')
        move_jointx_goal = MoveJointx.Request()
        move_jointx_goal.pos = [ x, y, z, 0.0, 0.0, 0.0 ]
        move_jointx_goal.vel = 30.0
        move_jointx_goal.acc = 30.0
        move_jointx_goal.sol = 2
        move_jointx_goal.ref = 1
        move_jointx_goal.mode = 1
        print('starting movement')
        future = self.move_jointx_client.call_async(move_jointx_goal)
        await future
        print(f'movement future {future.result()}')
           
    async def run_demo_loop(self):
        self.demo_task.cancel()    
        while True:
            await self.demo_loop()
           
    async def demo_loop(self):
        self.demo_task.cancel()      
        self.logger.info(f"{'*' * 5}Demo loop running{'*' * 5}")
        
        if not self.object_detector.service_is_ready():
            return self.logger.error('Service unavailable - object_detector')
        
        detection_request = ObjectPoseDetection.Request()
        detection_request.object_id = 'band'
        detection_request.aruco_id = 0
        
        detection_results: ObjectPoseDetection.Response = await self.object_detector.call_async(detection_request)
        self.logger.info(f'{detection_results=}')
        if not detection_results.success:
            return self.logger.warn(f'Not able to detect required object {detection_request.object_id}')
        
        target_x = detection_results.object_position.x * 1000.0
        target_y = detection_results.object_position.y * 1000.0
        target_z = detection_results.object_position.z * 1000.0

        if math.isnan(target_x) or math.isnan(target_y) or math.isnan(target_z):
            return self.logger.error(f'Detection error - invalid target {detection_results.object_position}')

        if target_z > self.PREP_RANGE:
            self.logger.info(f'Target far away, moving to closer location {target_z}')
            target_z = self.PREP_RANGE - 50

        self.logger.info(f'{target_x=}, {target_y=}, {target_z=}')
        await self.move_to_relative_goal(-target_x, target_y + 65.0, target_z - 150)

def main(args=None):
    rclpy.init(args=args)
    action_node = FollowObjects()
    # Run the executor within the asyncio event loop
    loop = asyncio.get_event_loop()
    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    loop.run_until_complete(rclpy.spin(action_node, executor=executor))
    action_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()