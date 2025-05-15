"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 

"""
# ROS imports
import rclpy
from rclpy.node import Node
# ROS Interface Import
from std_msgs.msg import String
from alex_interfaces.msg import ModeStatus
from alex_interfaces.srv import ControlMode
from action_msgs.msg import GoalStatusArray, GoalStatus
# Utilities Import
from alex_utilities.common_utilities import change_case

class AlexModeRegulator(Node):

    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.initialize()

        self.set_publishers()
        self.set_subscribers()
        self.set_services()

        self.logger.info(f"{node_name} started running")
    
    def initialize(self):
        self.mode = ControlMode.Request.MODE_NONE
        self.available_origins = [getattr(ControlMode.Request, item) for item in dir(ControlMode.Request) if item.startswith("ORIGIN_")]
        self.available_modes   = [getattr(ControlMode.Request, item) for item in dir(ControlMode.Request) if item.startswith("MODE_")]
        self.admin_origin      = [getattr(ControlMode.Request, item) for item in dir(ControlMode.Request) if item.startswith("ADMIN_ORIGIN_")]

        self.active_afi_actions: dict[str, GoalStatus.status] = {}

    def set_services(self):
        self.srv = self.create_service(ControlMode, 'alex_mode_regulator', self.alex_mode_regulator_callback)

    def set_publishers(self):
        self.alex_mode_pub = self.create_publisher(ModeStatus, 'mode_publisher', 5)
    
    def set_subscribers(self):
        self.afi_acton_status_sub = self.create_subscription(GoalStatusArray, "/execute_afi/_action/status", self.afi_action_status_callback, 5)

    def afi_action_status_callback(self, data: GoalStatusArray):
        """Keeps track of the active AFI."""
        for status in data.status_list:
            value: GoalStatus = status
            goal_id = value.goal_info.goal_id.uuid.__hash__
            if value.status == GoalStatus.STATUS_EXECUTING:
                self.active_afi_actions[goal_id] = value.status
            elif value.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.active_afi_actions.pop(goal_id, None)

    def ros_publisher_decorator(func):
        """
        Publishes the current mode every time the service is called 
        Also based on the result.success sets the current mode and self.mode
        """
        def wrapper(self, request: ControlMode.Request , response: ControlMode.Response):
            result: ControlMode.Response = func(self, request, response)
            if result.success == True:
                result.current_mode = self.mode = request.mode
            else:
                result.current_mode = self.mode
            status = ModeStatus(
                reason = result.reason,
                success = result.success,
                origin = request.origin,
                current_mode = self.mode,
            )
            self.alex_mode_pub.publish(status)
            return result
        return wrapper

    @ros_publisher_decorator
    def alex_mode_regulator_callback(self, request: ControlMode.Request, response: ControlMode.Response):
        self.logger.info(f'Mode requested by {request.origin}. Current mode: {self.mode or "none"} > {request.mode or "none"}')

        if request.origin not in self.available_origins:
            response.reason = "Invalid origin"
            self.logger.error(response.reason)
            return response
        
        if request.mode not in self.available_modes:
            response.reason = "Invalid mode"
            self.logger.error(response.reason)
            return response
        
        if self.mode == request.mode:
            response.success = True
            response.reason = "Already in the same mode"
            return response
        
        # Handling request.mode: None
        if request.mode == ControlMode.Request.MODE_NONE:
            if self.active_afi_actions:
                response.reason = "Active AFI Detected. Should be completed before changing mode"
                self.logger.warning(response.reason)
                return response
            elif request.origin in self.admin_origin:
                # ADMIN can change to none
                response.success = True
                return response
            elif request.origin == self.mode:
                response.success = True
                return response
            else: 
                response.reason = "Permission Denied"
                self.logger.warning(response.reason)
                return response
        
        if self.mode != ControlMode.Request.MODE_NONE and request.origin != self.mode:
            response.reason = "Cant change to foreign mode"
            self.logger.warning(response.reason)
            return response
        
        if request.origin != request.mode:
            response.reason = "Mode Switch can only be to self"
            self.logger.warning(response.reason)
            return response

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AlexModeRegulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
