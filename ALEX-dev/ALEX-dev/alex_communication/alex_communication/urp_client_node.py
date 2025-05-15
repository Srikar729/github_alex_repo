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
from rcl_interfaces.msg import ParameterDescriptor
# ROS Interface Import
from alex_interfaces.msg import SocketMessage
# Python Import
import socketio
import socketio.exceptions
from json import loads, dumps
# Utilities Import
from alex_utilities.common_utilities import change_case
from alex_utilities.urp_auth_utilities import get_encoded_room_id

class UrpClient(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.parameters()
        self.initializers()

        self.set_subscribers()
        self.set_publishers()

    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('access_token', value='',         
                               descriptor=ParameterDescriptor(description="The access token for individual robot workstation"))
        self.declare_parameter('host',         value="urp.neuralfoundry.co.uk/api/", 
                               descriptor=ParameterDescriptor(description="The backend connection for socket connection"))

        # get ros 2 parameters
        self.access_token:str = self.get_parameter('access_token').value
        self.host:str         = self.get_parameter('host').value

    def initializers(self,):
        self.logger = self.get_logger()

        self.sio = socketio.Client(logger=False, engineio_logger=False)
        self.room_id = get_encoded_room_id(self.access_token)

        self.sio.on("connect", self.connected)
        self.sio.on("connect_error", self.connection_error)
        self.sio.on("disconnect", self.disconnected)
        self.sio.on("*", self.handle_socket_msg)

        self.sio.connect(self.host)
    
    def set_subscribers(self,):
        self.socket_msg = self.create_subscription(SocketMessage, "socket_sender", self.ros_to_socket_message_handler, 10)
    
    def set_publishers(self,):
        self.pub_socket_msg = self.create_publisher(SocketMessage, "socket_receiver", 10)
    
    def ros_to_socket_message_handler(self, msg:SocketMessage):
        data = loads(msg.msg)
        formatted = {
            'room': self.room_id,
            'topic': msg.channel,
            'data': data,
        }
        self.logger.debug("Emiting room message: "+str(formatted))
        try:
            self.sio.emit('robot_msg', formatted)
        except socketio.exceptions.BadNamespaceError as e:
            # Found this error while testing it locally, and backend was restarted.
            self.logger.error(f"Error while sending socket message: {e}")
    
    def connected(self):
        self.logger.info('Connection established')
        self.sio.emit('join_room', self.room_id)
    
    def connection_error(self, data):
        self.logger.warning(f"[Connection Error] {data}")

    def disconnected(self):
        self.logger.warning('Disconnected from server')
    
    def handle_socket_msg(self, topic, message):
        handled_topics = [
            "manage_pause_resume",
            "cmd_ph",
            "cmd_container",
            "cmd_liquid_doser",
            "cmd_weight_balance",
            "cmd_raven",
            "cmd_magnetic_stirrer",
            "cmd_centrifuge",
            "cmd_control_mode",
        ]

        if topic == "execute_afi":
            self.ros_publish("execute_afi", message)
        elif topic in handled_topics:
            self.ros_publish(topic, self.json_stringify(message))
        else:
            self.logger.warning(f"Topic={topic} which is not handled yet.")
    
    @staticmethod
    def json_stringify(json_data) -> str:
        data = dumps(json_data)
        new_data = data.replace("'", '"')
        return new_data

    def ros_publish(self, channel:str, msg):
        data = SocketMessage()
        data.channel = channel
        data.msg = msg
        self.pub_socket_msg.publish(data)
    
def main(args=None):
    rclpy.init(args=args)
    node = UrpClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
