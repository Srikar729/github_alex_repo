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
from rclpy.client import Client
# ROS Interface Import
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
from time import sleep

class LifecycleNodeWaiter(Node):
    """Waits for the given lifecycle node to reach the required target state"""
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.declare_parameter("node_name", "config_fetcher")
        self.target_node_name = self.get_parameter("node_name").value
        
        self.declare_parameter("target_state", "PRIMARY_STATE_FINALIZED")
        target_state = self.get_parameter("target_state").value
        self.target_state = getattr(State, target_state)

        self.wait_target()
    
    def wait_target(self,):
        service = self.wait_for_client()

        self.wait_for_target_state(service, self.target_state)

    def wait_for_target_state(self, service: Client, target_state: int, attempt: int =5):
        for i in range(attempt):
            current_state = self.get_current_state(service)
            self.logger.info(f"Got {current_state=}")

            if current_state.id != self.target_state:
                self.logger.info(f"Not reached the required state yet, attempt={i/attempt}")
                sleep(2)
                continue

            self.logger.info("Reached the required state")
            return True

        return False
    
    def get_current_state(self, service: Client):
        self.logger.info("Getting the current lifecycle state")

        request = GetState.Request()
        future = service.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        
        response:GetState.Response = future.result()
        return response.current_state

    def wait_for_client(self):
        lifecycle_node_name = f"/{self.target_node_name}/get_state"
        change_state_srv = self.create_client(GetState, lifecycle_node_name)

        self.logger.info("Waiting for the target lifecycle client")

        is_ready = change_state_srv.wait_for_service(10)
        if not is_ready:
            raise Exception("The LifeCycle node is not ready, even after waiting for 10 secs.")
        return change_state_srv
    
def main(args=None):
    rclpy.init(args=args)
    lifecycle_listener = LifecycleNodeWaiter()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

