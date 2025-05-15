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
from rclpy.parameter import ParameterType
# ROS Interface Import
from alex_interfaces.msg import MqttMessage
from alex_interfaces.srv import MqttAttributes
# Python Import
import json
from requests import get
from bidict import bidict
import paho.mqtt.client as mqtt
from paho.mqtt.client import MQTTMessage as MQTTClientMessage
from paho.mqtt.enums import ConnackCode, MQTTErrorCode
# Utilities Import
from alex_utilities.common_utilities import change_case

class MqttClient(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.parameters()
        self.get_token_from_params()
        
        self.set_publishers()
        self.set_subscribers()
        self.set_services()
        self.initialize()

        self.logger.info(f"{node_name} is running")
 
    def parameters(self,):
        # declare ros 2 parameters
        self.declare_parameter('access_token', value='',   descriptor=ParameterDescriptor(description="The access token for individual robot workstation"))
        self.declare_parameter('devices_list', value=[""], descriptor=ParameterDescriptor(description="Name of the devices to be connected and get the token"))
        self.declare_parameter('host',         value='',   descriptor=ParameterDescriptor(description="The backend connection for MQTT connection"))
        self.declare_parameter('port',         value=1883, descriptor=ParameterDescriptor(description="MQTT connection port"))
        self.declare_parameter('api_host',     value='',   descriptor=ParameterDescriptor(description="The IOT MQTT backend"))

        # get ros 2 parameters
        self.key_device_token:str   = self.get_parameter('access_token').value
        self.devices_list:list[str] = self.get_parameter('devices_list').value
        self.host:str               = self.get_parameter('host').value
        self.port:int               = self.get_parameter('port').value
        self.api_host:str           = self.get_parameter('api_host').value
    
    def get_token_from_params(self,):
        parameter_delcaration = [ (
                device, 
                ParameterType.PARAMETER_NOT_SET, 
                ParameterDescriptor(
                    description=f"Device access token: {device}",
                    dynamic_typing=True,
                )
            ) for device in self.devices_list ]

        self.declare_parameters("", parameter_delcaration)
        self.device_tokens = [ param.value for param in self.get_parameters(self.devices_list)]
    
    def set_publishers(self,):
        self.pub_mqtt_msg = self.create_publisher(MqttMessage, "mqtt_receiver", 10)

    def set_subscribers(self,):
        self.mqtt_msg = self.create_subscription(MqttMessage, "mqtt_sender", self.ros_to_mqtt_handler, 10)
    
    def set_services(self,):
        self.mqtt_attributes_srv = self.create_service(MqttAttributes, "mqtt_attributes", self.handle_mqtt_attributes)
    
    def initialize(self,):
        self.mqtt_clients:bidict[str, mqtt.Client] = bidict()
        self.mqtt_clients["key-device"] = self._create_mqtt_client(self.key_device_token)
        for key, value in zip(self.devices_list, self.device_tokens):
            self.mqtt_clients[key] = self._create_mqtt_client(value)
        self.mqtt_start()

    def _create_mqtt_client(self, access_token):
        client = mqtt.Client()
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_connect_fail = self._on_connect_fail
        client.on_publish = self._on_publish
        client.on_message = self._on_message
        client.username_pw_set(access_token)
        client.connect(self.host, self.port, 60)
        return client
   
    def mqtt_start(self):
        for client in self.mqtt_clients.inv:
            client.loop_start()
    
    def subscribe_to_topic(self, client:mqtt.Client, topic:str):
        client.subscribe(topic)

    def handle_mqtt_attributes(self, request: MqttAttributes.Request, response: MqttAttributes.Response):
        self.logger.info(f"Got MQTT request for attribute={request.attribute_key}")

        all_devices = { key: value for key, value in zip(self.devices_list, self.device_tokens) }
        all_devices["key-device"] = self.key_device_token

        if request.device_id == MqttAttributes.Request.DEVICE_ALL:
            request_device = all_devices
        elif request.device_id not in all_devices:
            response.reason = f"{request.device_id} not in the devices list"
            return response
        else:
            request_device = { request.device_id: all_devices[request.device_id] }

        response_dict: dict[str] = {}

        for device_id, token in request_device.items():
            api_url   = f"{self.api_host}api/v1/{token}/attributes"
            parameter = {"sharedKeys": request.attribute_key}
            api_response = get(api_url, params=parameter)
            if not api_response.ok:
                self.logger.error(f"Error in getting response for {device_id}")
                continue
            api_response_json: dict = api_response.json()
            if shared:=api_response_json.get("shared"): pass
            else:
                self.logger.error(f"Could not find shared attribute for {device_id=}")
                continue
            if attribute:=shared.get(request.attribute_key): pass
            else:
                self.logger.error(f"Could not find {request.attribute_key=} for {device_id=}")
                continue

            response_dict.update({device_id: attribute})

        response.success = True
        response.attributes = json.dumps(response_dict)
        return response

    def _on_connect(self, client, userdata, flags, rc):
        if device_id:=self.mqtt_clients.inv.get(client):
            pass
        else: # Guard case
            self.logger.error("Got invalid client for device. Unknown device connected")
            return
        connection_status = ConnackCode(rc)
        self.logger.info(f"{device_id=} connected with result code: {connection_status.name}")
        if connection_status != ConnackCode.CONNACK_ACCEPTED:
            return
        self.subscribe_to_topic(client, 'v1/devices/me/rpc/request/+')
    
    def _on_disconnect(self, client:mqtt.Client, userdata, rc):
        error_code = MQTTErrorCode(rc)
        if device_id:=self.mqtt_clients.inv.get(client):
            self.logger.warning(f"The MQTT {device_id=} disconnected with {error_code=}")
        else: # Guard case
            self.logger.error("Got invalid client for device. Unknown device disconnected")
            return

    def _on_connect_fail(self, client:mqtt.Client, userdata):
        if device_id:=self.mqtt_clients.inv.get(client):
            self.logger.error(f"MQTT {device_id=} failed to connect.")
        else: # Guard case
            self.logger.error("Got invalid client for device. How did you get here??")
            return

    def _on_publish(self, client:mqtt.Client, userdata, mid):
        """Can be used to track each published message.
        """
        pass

    def _on_message(self, client, userdata, msg:MQTTClientMessage):
        if device_id:=self.mqtt_clients.inv.get(client):
            pass
        else: # Guard case
            self.logger.error("Got invalid client for device. How did you get here??")
            return
        decoded = msg.payload.decode()
        self.logger.debug(f'Device: {device_id}\nMessage: {decoded}')
        mqtt_message = MqttMessage()
        mqtt_message.device_id = device_id
        mqtt_message.msg = decoded
        self.pub_mqtt_msg.publish(mqtt_message)
    
    def ros_to_mqtt_handler(self, data:MqttMessage):
        self.logger.debug(f"{data.device_id=} {data.msg=}")
        if client:=self.mqtt_clients.get(data.device_id):
            client = self.mqtt_clients[data.device_id]
            client.publish(f'v1/devices/me/telemetry', data.msg)
        else:
            self.logger.error(f"Unable to find the {data.device_id}. Verify who you are sending to.")

def main(args=None):
    rclpy.init(args=args)
    node = MqttClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
