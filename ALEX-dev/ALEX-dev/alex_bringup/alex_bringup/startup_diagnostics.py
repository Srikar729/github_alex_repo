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
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from alex_interfaces.msg import ActiveNodesStatus
from alex_interfaces.srv import GripperCommand
# Python Import
import time 
from asyncio import get_event_loop
from typing import Callable, Iterable
# Utilities Import
from alex_utilities.common_utilities import change_case

class StartupDiagnostics(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        
        self.initilize()
        self.subscriber()

        self.start_up_timer = self.create_timer(5.0, self.start_run_sequence) # Delaying the start of the node by 5 seconds
    
    def initilize(self,):
        self.current_active_nodes: list[str] = []
        self.start_up_sequencing: dict[str, Callable | list[Callable]] = {
            # Add any node dependency and link its function
            "/robot_9a6367ad/doosan_gripper_control": self.doosan_gripper_reset, # BUG: Hardcoded device, should come from the Thingsboard
        }
    
    def subscriber(self):
        self.active_node_status = self.create_subscription(
            ActiveNodesStatus, 
            "active_node_status", 
            self.handle_active_node_status, 
            5, 
            callback_group=ReentrantCallbackGroup(),
        )
    
    def shutdown(self,):
        self.logger.info("Sequence Completed. Shuting Down")
        self.destroy_node()
        self.logger.info("Node Dead....")
        exit()

    async def doosan_gripper_reset(self,):
        doosan_arm_control = self.create_client(GripperCommand, "/robot_9a6367ad/gripper_control", callback_group=ReentrantCallbackGroup())
        if not doosan_arm_control.wait_for_service(10.0):
            self.logger.info("Timed out waiting for `doosan_arm_control`")
            return
        self.logger.info("Closing gripper")
        future = doosan_arm_control.call_async(GripperCommand.Request(command=GripperCommand.Request.PICK))
        await future
        self.logger.info("Opening gripper")
        future = doosan_arm_control.call_async(GripperCommand.Request(command=GripperCommand.Request.DROP))
        await future

    async def start_run_sequence(self,):
        self.start_up_timer.cancel()
        for node_name, func in self.start_up_sequencing.items():
            while node_name not in self.current_active_nodes:
                if not rclpy.ok(): return
                self.logger.info(f"waiting for {node_name} to be active")
                # Wait for node to become active
                time.sleep(5)
            if not isinstance(func, Iterable):
                func = [func]
            for func_item in func:
                self.logger.info(f"Executing the function for {node_name} === {func_item.__name__}()")
                await func_item()
                #TODO: Should publish to a topic as well
        self.shutdown()

    def handle_active_node_status(self, data: ActiveNodesStatus):
        self.logger.debug("got active node status")
        self.current_active_nodes.clear()
        self.current_active_nodes.append("bringup") # So when something that needs to run in beginning can run

        for status in data.node_status:
            status.package_name
            for node_ in status.nodes:
                if not node_.is_active: 
                    continue
                self.current_active_nodes.append(node_.node_name)

def main(args=None):
    rclpy.init(args=args)
    node = StartupDiagnostics()
    loop = get_event_loop()
    executor = MultiThreadedExecutor(num_threads=2)
    loop.run_until_complete(rclpy.spin(node, executor=executor))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
