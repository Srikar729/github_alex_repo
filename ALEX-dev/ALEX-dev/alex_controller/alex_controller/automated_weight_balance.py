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
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from rcl_interfaces.msg import ParameterDescriptor
from alex_interfaces.msg import Weight
from alex_interfaces.srv import DCActuator, WeightCommand, MeasureWeights
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import

class AutomatedWeightBalanceControlNode(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.initialize()
        self.parameters()
        self.create_service_client()
        self.set_services()
        self.set_subscribers()

        self.logger.info(f"{node_name} started running")
        self.logger.info(f"This weight balance is registered {'with' if self.has_door else 'without'} door")
    
    def initialize(self,):
        pass

    def parameters(self,):
        self.declare_parameter('has_door',     value=True, descriptor=ParameterDescriptor(description="Door setup connected?"))

        self.has_door:bool       = self.get_parameter('has_door').value

    def create_service_client(self,):
        self.start_weight_balance_srv = self.create_client(WeightCommand, "weight_balance/start_weight_balance")
        if not self.has_door:
            self.linear_actuator_srv      = self.create_client(DCActuator, "actuator/move_actuator")

    def set_services(self,):
        self.create_service(MeasureWeights, "measure_weight_balance", self.handle_weight_balance, callback_group=ReentrantCallbackGroup())
    
    def set_subscribers(self,):
        pass
    
    async def handle_weight_balance(self, request:MeasureWeights.Request, response:MeasureWeights.Response):
        self.logger.info(f"Got request command: {request.command} weight_mode: {request.weight_mode}")
        match request.command:
            case MeasureWeights.Request.COMMAND_DOOR_OPEN | MeasureWeights.Request.COMMAND_DOOR_CLOSE:
                if not self.has_door:
                    response.success = True
                    response.reason = "Door not connected"
                    return response
                if not self.linear_actuator_srv.service_is_ready():
                    response.reason = "Actuator Service is not ready"
                    return response
                actuator_request = DCActuator.Request()
                actuator_request.direction = {
                    MeasureWeights.Request.COMMAND_DOOR_OPEN : DCActuator.Request.DIRECTION_FORWARD,
                    MeasureWeights.Request.COMMAND_DOOR_CLOSE : DCActuator.Request.DIRECTION_BACKWARD,
                }[request.command]
                actuator_response_future = self.linear_actuator_srv.call_async(actuator_request)
                await actuator_response_future
                actuator_response: DCActuator.Response = actuator_response_future.result()
                match (actuator_response.status):
                    case DCActuator.Response.STATUS_COMPLETED:
                        response.success = True
                        return response
                    case DCActuator.Response.STATUS_CANCELLING:
                        response.reason = "Got invalid status. I should be be cancelling"
                    case DCActuator.Response.STATUS_ERROR:
                        response.reason = actuator_response.reason
                    case DCActuator.Response.STATUS_CANCELLED:
                        response.reason = actuator_response.reason
            case MeasureWeights.Request.COMMAND_MEASURE:
                if not self.start_weight_balance_srv.service_is_ready():
                    response.reason = "Weight Balance Service is not ready"
                    return response
                measure_request = WeightCommand.Request()
                measure_request.job_id = measure_request.job_id
                measure_request.mode = measure_request.mode
                measure_response_future = self.start_weight_balance_srv.call_async(measure_request)
                await measure_response_future
                measure_response:WeightCommand.Response = measure_response_future.result()
                response.result = measure_response.data
                match measure_response.data.status:
                    case Weight.COMPLETED:
                        response.success = True
                        return response
                    case Weight.ERROR:
                        response.reason = measure_response.data.reason
            case _:
                response.reason = "Invalid command"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedWeightBalanceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
