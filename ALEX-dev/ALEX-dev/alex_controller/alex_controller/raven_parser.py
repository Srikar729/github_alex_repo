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
from rclpy.callback_groups import ReentrantCallbackGroup
# ROS Interface Import
from std_msgs.msg import String
from alex_interfaces.msg import ModeStatus
from alex_interfaces.srv import ControlMode
from rcl_interfaces.msg import ParameterDescriptor
from dsr_msgs2.srv import MoveJointx, SetToolDigitalOutput
# Python Import
import json
from scipy.spatial.transform import Rotation

class RavenParser(Node):
    def __init__(self):
        super().__init__('raven_subscriber')
        self.logger = self.get_logger()
        self.parameters()
        self.create_subscribers()
        self.create_service_clients()

        self.current_mode = ControlMode.Request.MODE_NONE

    def parameters(self):
        # declare ros 2 parameters
        self.declare_parameter('RAVEN_xy_treshold', value= 0.999,         
                               descriptor=ParameterDescriptor(description="x,y treshold value"))
        self.declare_parameter('RAVEN_z_treshold', value=0.200, 
                               descriptor=ParameterDescriptor(description="z treshold value"))
        self.declare_parameter('RAVEN_scale_factor', value=1000, 
                               descriptor=ParameterDescriptor(description="scale factor for unity to arm convertion"))
        self.declare_parameter('doosan_arm_velocity', value=180.0, 
                               descriptor=ParameterDescriptor(description="velocity value for doosan arm"))
        self.declare_parameter('doosan_arm_acc', value=100.0, 
                               descriptor=ParameterDescriptor(description="acc value for doosan arm"))

        # get ros 2 parameters
        self.xy_treshold = self.get_parameter('RAVEN_xy_treshold').value
        self.z_treshold = self.get_parameter('RAVEN_z_treshold').value
        self.scale_factor = self.get_parameter('RAVEN_scale_factor').value
        self.arm_velocity = self.get_parameter('doosan_arm_velocity').value
        self.arm_acc = self.get_parameter('doosan_arm_acc').value

    def create_subscribers(self):
        self.raven_msg_sub = self.create_subscription(String, 'raven_msg', self.raven_subscriber_callback, 10)
        self.mode_status_sub = self.create_subscription(ModeStatus, 'mode_publisher', self.mode_status_callback, 10)
    
    def create_service_clients(self):
        self.mode_permission_client = self.create_client(ControlMode, 'alex_mode_regulator', callback_group=ReentrantCallbackGroup())
        self.move_jointx_client = self.create_client(MoveJointx, '/dsr01/motion/move_jointx')
        self.tool_output_client = self.create_client(SetToolDigitalOutput,'/dsr01/io/set_tool_digital_output')

    def mode_status_callback(self, msg: ModeStatus):
        self.current_mode = msg.current_mode

    async def alex_controller_mode(self, setter: bool = True):
        if not self.mode_permission_client.service_is_ready():
            return False, "Service is not ready, to set alex mode"
        mode_request = ControlMode.Request()
        mode_request.origin = ControlMode.Request.ORIGIN_RAVEN
        if setter:
            mode_request.mode = ControlMode.Request.MODE_RAVEN
        else:
            mode_request.mode = ControlMode.Request.MODE_NONE
        mode_response = self.mode_permission_client.call_async(mode_request)
        await mode_response
        mode_result:ControlMode.Response = mode_response.result()
        if not mode_result.success:
            return False, mode_result.reason
        self._alex_controller_mode = mode_result.current_mode
        self.logger.info("Got alex controller mode permission to execute RAVEN")
        return True, "All's well"
        
    async def raven_subscriber_callback(self, msg: String):
        self.logger.info('RAVEN cmd received: "%s"' % msg.data)
        
        if not self.mode_permission_client.service_is_ready():
            return self.logger.error(f'mode_permission_client is not available')
        
        float_array: list[float] = json.loads(msg.data)
        if not isinstance(float_array, list):
            return self.logger.error(f'raven parser given invalid msg {float_array}')
        
        if not len(float_array):
            await self.alex_controller_mode(setter=False)
            return

        if self.current_mode != ControlMode.Request.MODE_RAVEN:
            success, error = await self.alex_controller_mode()
            if not success:
                self.logger.error(f'Did not get the permission with reason: {error}')
                return

        #checking actual data for warning before clamping
        if abs(float_array[0]) > self.xy_treshold:
            self.logger.warning(f'x value {float_array[0]} is outside the threshold of +/-{self.xy_treshold}')
        if abs(float_array[1]) > self.xy_treshold:
            self.logger.warning(f'y value {float_array[1]} is outside the threshold of +/-{self.xy_treshold}')
        if abs(float_array[2] - self.z_treshold) > self.xy_treshold:
            self.logger.warning(f'z value {float_array[2]} is outside the threshold of +/-{self.xy_treshold} (adjusted by z_treshold)')
            
        #clamp
        x = round(max(min(float_array[0], self.xy_treshold), -self.xy_treshold) * self.scale_factor)
        y = round(max(min(float_array[1], self.xy_treshold), -self.xy_treshold) * self.scale_factor)
        z = round(max(min(float_array[2] - self.z_treshold, self.xy_treshold), self.z_treshold) * self.scale_factor)

        A, B, C = self.convert_unity_to_euler(float_array[3], float_array[4], float_array[5], float_array[6])
        
        pos = [float(num) for num in [x, y, z, A, B, C]]
        self.logger.info(f'RAVEN cmd to new pos {pos}')
        move_jointx_goal = MoveJointx.Request()
        move_jointx_goal.pos = pos
        move_jointx_goal.vel = self.arm_velocity
        move_jointx_goal.acc = self.arm_acc
        move_jointx_goal.sol = 2
        move_jointx_goal.sync_type = 1
        try:
            self.move_jointx_client.call_async(move_jointx_goal)
            if len(float_array) > 7:
                gripper_value = float_array[7]
                if gripper_value == 1.0:
                    self.tool_output_client.call_async(SetToolDigitalOutput.Request(index = 1, value = 1))
                    self.tool_output_client.call_async(SetToolDigitalOutput.Request(index = 2, value = 1))
                else:
                    self.tool_output_client.call_async(SetToolDigitalOutput.Request(index = 1, value = 0))
                    self.tool_output_client.call_async(SetToolDigitalOutput.Request(index = 2, value = 0))     
        except Exception as e:
            self.logger.error(f"failed movement with exception {e}")  

    def convert_unity_to_euler(self,x, y, z, w):
        """Convert quaternion to Euler angles."""
        inv_X = -x
        q = Rotation.from_quat([inv_X, y, z, w])
        abc = q.as_euler('zyz', degrees=True)
        return abc[0], abc[1], abc[2]

def main(args=None):
    rclpy.init(args=args)
    node = RavenParser()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
