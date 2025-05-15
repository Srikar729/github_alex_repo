"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 
"""

"""
Docs: Radwag Analytical Balance
Command Document : https://radwag.com/pdf2/en/R2_IMMU-03-51-06-23-EN.pdf
Page: 88
(14.1.1) - Response Format
"""
# ROS Import
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
# ROS Interface Import
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from alex_interfaces.srv import WeightCommand
from alex_interfaces.msg import Weight, SerialMessage
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
from time import sleep

class WeightBalance(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.parameters()
        self.initialize()
        self.service_client()
        self.set_publishers()
        self.set_subscriber()
        self.set_services()
        self.configure_device()
        self.logger.info(f"{node_name} started running")

    def parameters(self):
        # declare ros 2 parameters
        self.declare_parameter('weight_stabilizing_duration', 10)
        # get ros 2 parameters
        self.weight_stabilizing_duration:int = self.get_parameter('weight_stabilizing_duration').value

        self.weight_stabilizing_duration:Duration = Duration(seconds=self.weight_stabilizing_duration)
    
    def initialize(self):
        self.weight_msg = Weight(
            status = Weight.ERROR,
            reason = "Device not activated yet"
        )

    def service_client(self):
        serial_node_name = self.get_namespace()+"/serial_interface_lifecycle/change_state"
        self.change_state_srv = self.create_client(ChangeState, serial_node_name)

    def set_publishers(self):
        self.pub_weight_status = self.create_publisher(Weight, "weight_data", 5)

    def set_subscriber(self):
        self.create_subscription(SerialMessage, "from_serial", self.from_serial_callback, 10, 
                                 callback_group=ReentrantCallbackGroup())

    def set_services(self):
        self.create_service(WeightCommand, "start_weight_balance", self.handle_weight_command)
    
    # ---- Serial Device setup -----
    def configure_device(self,):
        self.logger.info("Configuring Device")
        wait_time = 20
        for i in range(wait_time):
            if self.change_state_srv.wait_for_service(1):
                break
            self.logger.info(f"Serial lifecycle service waiting. Waiting: {wait_time - i}")
        else:
            self.logger.error("Could not find the device serial node, try again..")
            return
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.change_state_srv.call_async(change_state_req)
        future.add_done_callback(self.configure_callback)
    
    def configure_callback(self, future: Future):
        result: ChangeState.Response = future.result()
        if not result.success:
            self.logger.error("Failed to activate the device. Will try again in 2 seconds")
            sleep(2)
            self.configure_device()
            return
        self.logger.info("Configuring Device: Success")
        self.activate_device()
    
    def activate_device(self,):
        self.logger.info("Activating Device")
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.change_state_srv.call_async(change_state_req)
        future.add_done_callback(self.activate_callback)

    def activate_callback(self, future: Future):
        result: ChangeState.Response = future.result()
        if not result.success:
            self.logger.error("Failed to activate the device. Will try again in 2 seconds")
            sleep(2)
            self.activate_device()
            return
        self.logger.info("Activating Device: Success")
    # ---- Serial Device setup:END -----

    def from_serial_callback(self, msg: SerialMessage):
        self.logger.debug(f"Received weight data: {msg.data}")
        weight_msg = Weight()
        if msg.state != SerialMessage.CONNECTED:
            self.weight_msg.status = Weight.ERROR
            self.weight_msg.reason = f"Device disconnected"
            # TODO: Should this be send to mqtt as error?
            self.logger.warning(f"Device Disconnected.")
            return
        if not msg.data:
            return

        stability_marker_lookup = {
            " " : True, # Stable       
            "?" : False,  # Unstable
            "^" : False, # High Limit
            "v" : False, # Low Limit
        }
        
        command          = msg.data[0:2]
        stability_marker = msg.data[3]
        sign             = msg.data[5]
        mass             = msg.data[6:15]
        space            = msg.data[15]
        unit             = msg.data[16:18]

        weight_msg.value.value = float(sign + mass.strip())
        weight_msg.value.unit = unit.strip()
        weight_msg.stable = stability_marker_lookup.get(stability_marker, False)
        weight_msg.status = Weight.COMPLETED

        self.weight_msg = weight_msg
    
    def ros_publisher_decorator(func):
        def wrapper(self, request:WeightCommand.Request , response:WeightCommand.Response):
            result:WeightCommand.Response = func(self, request, response)
            result.data.job_id = request.job_id
            self.pub_weight_status.publish(result.data)
            return result
        return wrapper

    @ros_publisher_decorator
    def handle_weight_command(self, request: WeightCommand.Request, response: WeightCommand.Response):
        """Handle incoming weight command requests."""
        self.logger.info(f"Received request, {request.mode=}")
        if request.mode not in [WeightCommand.Request.MODE_STABLE, WeightCommand.Request.MODE_LATEST]:
            response.success = False
            response.data.status = Weight.ERROR
            response.data.reason = "Invalid mode"
            return response

        if self.weight_msg.status != Weight.COMPLETED:
            response.success = False
            response.data = self.weight_msg
            return response

        if request.mode == WeightCommand.Request.MODE_STABLE:
            self.logger.info(f"Waiting for stable weight data from device...")
            start_time = self.get_clock().now()

            # Wait for stable data
            while not self.weight_msg.stable:
                sleep(0.5)
                if (time_elasped:=self.get_clock().now() - start_time) > self.weight_stabilizing_duration:
                    self.logger.error("Timeout waiting for stable data.")
                    response.success = False
                    response.data = self.weight_msg
                    response.data.status = Weight.ERROR
                    response.data.reason = "Timeout waiting for stable data."
                    return response
                if self.weight_msg.status != Weight.COMPLETED:
                    response.success = False
                    response.data = self.weight_msg
                    return response
                self.logger.info(f"Waiting for stable weight data: {(time_elasped.to_msg().sec - self.weight_stabilizing_duration.to_msg().sec)}")

        # Prepare the response with the latest data
        response.success = True
        response.data = self.weight_msg
        self.logger.info(f"Response: weight={response.data.value.value} "
                                    f"unit={response.data.value.unit} "
                                    f"stable={response.data.stable} "
                                    f"tared={response.data.tared} " )
        return response

def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = WeightBalance()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
