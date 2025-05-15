# ROS Import
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# ROS Interface Imports
import message_filters
from sensor_msgs.msg import Image
from alex_interfaces.srv import ObjectPoseDetection
# Utilities Import
from alex_utilities.common_utilities import change_case
#python imports 
import cv2
import time
import numpy as np
# Detection Class
from alex_intelligence.object_detector import ObjectDetector3D

class DetectionService(Node):

    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.debug = True
        self.parameters()
        self.initialize()
        self.init_subscribers()
        self.set_services()
        self.rgb_frame: np.ndarray = None
        self.depth_frame: np.ndarray = None

    def parameters(self):
        # declare ros 2 parameters
        #declare a parameter for model path
        self.declare_parameter('model_path', value="alex_intelligence/resource/lab_v3.pt", descriptor=ParameterDescriptor(description="The path for the model weights."))
        
        #declare a parameter for camera_params
        # TODO see how to get these dynamically?
        self.declare_parameter('camera_params', value=[733.0479125976562,733.0479125976562,636.56005859375,350.62615966796875], descriptor=ParameterDescriptor(description="The path for the model weights."))

        # get ros 2 parameters
        self.model_path: str = self.get_parameter('model_path').value
        self.camera_params_list: list = self.get_parameter('camera_params').value
        self.camera_params: dict = {'fx':self.camera_params_list[0],'fy':self.camera_params_list[1],'ppx':self.camera_params_list[2],'ppy':self.camera_params_list[3]}
    
    def initialize(self):
        self.logger.info(f'initilizing object detector node, model: {self.model_path}, params {self.camera_params}')
        self.bridge = CvBridge()
        self.object_detection = ObjectDetector3D(self.model_path, self.camera_params, debug=self.debug)

    def init_subscribers(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.rgb_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/zed/zed_node/left_raw/image_raw_color', 
            qos_profile=qos_profile,
        )
        self.depth_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/zed/zed_node/depth/depth_registered', 
            qos_profile=qos_profile,
        )

        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10)
        self.ts.registerCallback(self.synchronized_callback)
    
    def synchronized_callback(self, rgb_msg, depth_msg):
        self.logger.debug("got frame")
        self.rgb_frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        # TODO remove
        # if self.debug:
        #     try:
        #         cv2.imwrite(f"alex_intelligence/resource/rgb/rgb_{time.time}.png", self.rgb_frame)
        #         cv2.imwrite(f"alex_intelligence/resource/depth/depth_{time.time}.png", self.depth_frame)
        #     except Exception as e:
        #         print("expection as error", e)

    def set_services(self):
        self.create_service(ObjectPoseDetection, "object_detector", self.detections)
        
    def _return_failure(self, response: ObjectPoseDetection.Response) -> None:
        response.success = False
        response.object_position.x = 0.0
        response.object_position.y = 0.0
        response.object_position.z = 0.0
        return response

    def detections(self, request: ObjectPoseDetection.Request, response: ObjectPoseDetection.Response):
        self.logger.info(f"got request {request.object_id}")
        request.object_id
        if self.rgb_frame is None or self.depth_frame is None:
            self.logger.error('No frame available to detect')
            return self._return_failure(response)
        
        point = self.object_detection.process(self.rgb_frame, self.depth_frame, request.object_id)
        if point is None:
            self.logger.error('Requested object not detected in frame')
            return self._return_failure(response)
        x, y, z = point
        self.logger.debug(f'Detected requested object {x=}, {y=}, {z=}')
        response.success = True
        response.object_position.x = float(x)
        response.object_position.y = float(y)
        response.object_position.z = float(z)
        return response
            
def main():
    rclpy.init()
    detection_node = DetectionService()
    rclpy.spin(detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()