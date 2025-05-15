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
from rclpy.subscription import Subscription
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleNode, LifecycleState, LifecyclePublisher
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
# ROS Interface Import
from alex_interfaces.msg import SerialMessage
from lifecycle_msgs.msg import State
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
import serial
from time import sleep
from threading import Thread
from serial.tools import list_ports
from enum import Enum

class ReadType(Enum):
    UTF8 = "utf-8"
    HEX  = 'hex'
    
    @classmethod
    def _missing_(cls, value):
        return cls.UTF8
    
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_

class SerialInterfaceLifecycle(LifecycleNode):
    def __init__(self, **kwargs):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name, **kwargs)
        self.logger = self.get_logger()
        
        # Declaring variables for params
        self.serial_number    = ""
        self.device_port      = ""
        self.baud_rate        = 115200
        self.firmware_version = "0.1"
        self.timeout          = 5
        self.read_type        = "utf-8"
        self.parity           = serial.PARITY_NONE
        self.bytesize         = serial.EIGHTBITS
        self.write_terminator = "\n"
        self.read_terminator  = "\n"
        # Declaring params to ros2
        self._declare_parameters = dict(
            serial_number    = self.serial_number,
            device_port      = self.device_port,
            baud_rate        = self.baud_rate,
            firmware_version = self.firmware_version,
            timeout          = self.timeout,
            read_type        = self.read_type,
            read_terminator  = self.read_terminator,
            write_terminator = self.write_terminator,
        )
        self._declare_parameters.items()
        self.declare_parameters('', self._declare_parameters.items())
        # Declaring variables
        self.pub_from_serial:  LifecyclePublisher = None
        self.sub_to_serial:          Subscription = None
        # Declaring control variables
        self.serial_connection = serial.Serial()
        self.reconnect_serial_timer = self.create_timer(2.0, self.reconnect_serial)
        self.reconnect_serial_timer.cancel()
        self.should_read: bool = False
        self.logger.info(f"{node_name} is running")
        
    def get_device_port(self, serial_number: str):
        com_ports = list_ports.comports()
        self.logger.info(f"Device serial number: '{serial_number}'")
        device, *_ = [ com_port for com_port in com_ports 
                                    if com_port.serial_number == serial_number ] or [None]
        if not device:
            available_serial_port = [devices.serial_number for devices in com_ports 
                                     if devices.serial_number]
            self.logger.error("Could not find the required device serial number, "
                              f"Check the connection. Available {available_serial_port}")
            return None
        return device.device
        
    def is_device_connected(self, device_port:str):
        if not device_port: 
            return False
        devices = list_ports.comports()
        return any( device.device == device_port for device in devices)
        
    def reconnect_serial(self, ):
        self.reconnect_serial_timer.cancel()
        self.logger.info("Starting reconnection procedure")
        if self._state_machine.current_state[0] == State.PRIMARY_STATE_ACTIVE:
            status = self.trigger_deactivate()
            if status != TransitionCallbackReturn.SUCCESS:
                self.logger.error("Some error while `node deactivate`, Could not reconnect")
                return
            self.logger.info("[Reconnection] Node deactivated")
        if self._state_machine.current_state[0] == State.PRIMARY_STATE_INACTIVE:
            status = self.trigger_cleanup()
            if status != TransitionCallbackReturn.SUCCESS:
                self.logger.error("Some error while `node cleanup`, Could not reconnect")
                return
            self.logger.info("[Reconnection] Node cleaned up")
        status = self.trigger_configure()
        if status != TransitionCallbackReturn.SUCCESS:
            self.logger.error("Some error while `node configure`, reconnecting...")
            self.reconnect_serial_timer.reset()
            return
        self.logger.info("[Reconnection] Node Configured")
        status = self.trigger_activate()
        if status != TransitionCallbackReturn.SUCCESS:
            self.logger.error("Some error while `node activate`, reconnecting...")
            msg = SerialMessage()
            msg.state = SerialMessage.ACTIVATING
            self.pub_from_serial.publish(msg)
            self.reconnect_serial_timer.reset()
            return
        msg = SerialMessage()
        msg.state = SerialMessage.CONNECTED
        self.pub_from_serial.publish(msg)
        self.logger.info("[Reconnection] Node Activated. Reconnection successful.")

    def to_serial_callback(self, msg:SerialMessage):
        if self.pub_from_serial is None or not self.pub_from_serial.is_activated:
            self.logger.warning("Lifecycle publisher is currently inactive. Ignoring the subscribed message.")
        
        if self.read_type == ReadType.UTF8:
            msg.data += self.write_terminator
            data = msg.data.encode('utf-8')
        elif self.read_type == ReadType.HEX:
            data = bytes.fromhex(msg.data)
        try:
            self.serial_connection.write((data))
            self.logger.debug(f"Sent serial data: {data}")
        except Exception as e:
            self.logger.error(f'Error writing to serial port: {e}')
        
    def read_from_serial(self,):
        if self.pub_from_serial is None or not self.pub_from_serial.is_activated:
            self.logger.warning("Lifecycle publisher is currently inactive. Messages are not published.")
            sleep(0.5)   
            return
        msg = SerialMessage()
        try:
            # if self.serial_connection.in_waiting <= 0: return # Making sure there is incoming data in serial
            data = self.serial_connection.read_until(self.read_terminator)
            if self.read_type == ReadType.UTF8:
                data = data.decode('utf-8').strip()
            elif self.read_type == ReadType.HEX:
                data = data.hex()
        except Exception as e:
            self.logger.error(f'Error reading from serial port: {e}')
            msg.state = SerialMessage.DISCONNECTED
            self.pub_from_serial.publish(msg)
            self.stop_read_serial_thread(should_join=False)
            self.logger.error("Attempting start reconnection timer")
            self.reconnect_serial_timer.reset()
            return
        
        if not data:
            return
        
        msg.state = SerialMessage.CONNECTED
        msg.data  = data
        self.pub_from_serial.publish(msg)
        self.logger.debug(f'Publishing: {data}')
    
    def read_serial(self,):
        while self.should_read:
            self.read_from_serial()
        self.logger.info("Read Serial thread closed")
    
    def start_read_serial_thread(self,):
        self.should_read = True
        self.serial_read_thread = Thread(target=self.read_serial)
        self.serial_read_thread.start()
    
    def stop_read_serial_thread(self, should_join=True):
        self.should_read = False
        if should_join:
            self.serial_read_thread.join()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        # get ros2 parameters and set variables
        self._declare_parameters.keys()
        _parameters = self.get_parameters(self._declare_parameters.keys())
        for parameter in _parameters:
            setattr(self, parameter._name, parameter.value)
        if not (self.serial_number or self.device_port):
            self.logger.error(f"Failed to configure node, "
                               "serial_number or device_port is required to connect to device")
            return TransitionCallbackReturn.FAILURE        
        # Search for device ports
        if self.serial_number:
            self.device_port = self.get_device_port(self.serial_number)
        # Verify device port
        if not self.is_device_connected(self.device_port):
            self.logger.error(f"Failed to configure node, port not found: {self.device_port}")
            return TransitionCallbackReturn.FAILURE
        # Parameter setup
        if self.baud_rate not in serial.SerialBase.BAUDRATES:
            self.baud_rate = 115200
            self.logger.warning(f"Invalid {self.baud_rate=}, setting baud_rate:={self.baud_rate}")
        if self.parity not in serial.SerialBase.PARITIES:
            self.parity = serial.PARITY_NONE
            self.logger.warning(f"Invalid {self.parity=}, setting parity:={self.parity}")
        if self.bytesize not in serial.SerialBase.BYTESIZES:
            self.bytesize = serial.EIGHTBITS
            self.logger.warning(f"Invalid {self.bytesize=}, setting bytesize:={self.bytesize}")
        if not ReadType.has_value(self.read_type):
            self.logger.warning(f"Invalid {self.read_type=}, setting read_type:=utf-8")
        self.read_type = ReadType(self.read_type)
        # Minior modification
        self.read_terminator = self.read_terminator.encode()
        # Setting publishers
        self.pub_from_serial = self.create_lifecycle_publisher(SerialMessage, "from_serial", 10)
        # Setting subscribers
        self.sub_to_serial   = self.create_subscription(SerialMessage, "to_serial", self.to_serial_callback, 10)
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        try:
            self.serial_connection.port     = self.device_port
            self.serial_connection.baudrate = self.baud_rate
            self.serial_connection.timeout  = self.timeout
            self.serial_connection.parity   = self.parity
            self.serial_connection.stopbits = serial.STOPBITS_ONE
            self.serial_connection.bytesize = self.bytesize
            self.serial_connection.open()
        except serial.SerialException as e:
            self.logger.error(f'Error connecting to serial port: {e}')
            self.serial_connection.close()
            return TransitionCallbackReturn.FAILURE
        self.start_read_serial_thread()
        return super().on_activate(state)  
        
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self.stop_read_serial_thread()
        self.serial_connection.close()
        return super().on_deactivate(state)
        
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.serial_connection.close()
        self.destroy_lifecycle_publisher(self.pub_from_serial)
        self.destroy_subscription(self.sub_to_serial)
        return TransitionCallbackReturn.SUCCESS
        
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        self.serial_connection.close()
        self.destroy_timer(self.reconnect_serial_timer)
        self.destroy_lifecycle_publisher(self.pub_from_serial)
        self.destroy_subscription(self.sub_to_serial)
        return TransitionCallbackReturn.SUCCESS
    
def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    lc_node = SerialInterfaceLifecycle()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        lc_node.should_read = False
        lc_node.destroy_node()

if __name__ == '__main__':
    main()
