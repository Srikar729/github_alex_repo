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
from rclpy.node import Node, Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# ROS Interface Import
from sensor_msgs.msg import Image
# Utilities Import
from alex_utilities.common_utilities import change_case
# Python Import
import cv2
from cv_bridge import CvBridge
from linuxpy.video.device import Device, iter_video_capture_devices

class ZedCameraImage(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.parameters()
        self.connect_camera()
        
        self.image_failure_count = 0
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.left_image_publisher = self.create_publisher(Image, '/left_image', qos_profile)
        self.right_image_publisher = self.create_publisher(Image, '/right_image', qos_profile)

    def parameters(self):
        # Declare parameters
        self.declare_parameter('video_device', '')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('conversion_type', 'COLOR_BGRA2RGB')

        # Retrieve parameters
        self.video_device:str    = self.get_parameter('video_device').value
        self.width:int           = self.get_parameter('width').value
        self.height:int          = self.get_parameter('height').value
        self.conversion_type:str = self.get_parameter('conversion_type').value

        # Validate parameters
        self.is_valid_video_device(self.video_device)    
        self.is_valid_resolution(self.width, self.height)
        self.is_valid_conversion_type(self.conversion_type)

        # Modifications
        self.video_device    = None if not self.video_device else self.video_device
        self.conversion_type = None if not self.conversion_type else getattr(cv2, self.conversion_type)

    def get_zed_camera_device(self, *, search_depth=10):
        for device in iter_video_capture_devices():
            with device as video:
                if video.info.card != "ZED-M: ZED-M": continue
                return video
            
        for cam_id in range(search_depth):
            cam_device = Device.from_id(cam_id)
            if not cam_device.filename.exists(): continue
    
    def connect_camera(self):
        # If not defined, look for zed camera
        if not self.video_device:
            video_device = self.get_zed_camera_device()
            self.video_device = video_device and video_device.filename
        
        if not self.video_device:
            raise Exception("No Zed Camera found")

        self.cap = cv2.VideoCapture(self.video_device)
        if not self.cap.isOpened():
            raise Exception("Failed to open camera!")

        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    
    def _publish_image(self, image: cv2.Mat, publisher: Publisher):
        imgmsg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        publisher.publish(imgmsg)

    def capture_and_publish(self):
        try:
            while rclpy.ok():
                # Process events (e.g., shutdown)
                rclpy.spin_once(self, timeout_sec=1/20) # 20 fps

                ret, frame = self.cap.read()
                if not ret:
                    self.image_failure_count += 1
                    self.logger.warning("Failed to capture image")
                    if self.image_failure_count > 20:
                        raise Exception("Failed to capture image in 20 attempts")
                    continue
                self.image_failure_count = 0

                # Convert the frame based on the specified conversion type
                if self.conversion_type:
                    frame = cv2.cvtColor(frame, self.conversion_type)
                
                # Split image
                right_image = frame[:,frame.shape[1]//2 : frame.shape[1]   ]
                left_image  = frame[:,        0         : frame.shape[1]//2]
                
                # Convert OpenCV image to ROS Image message
                self._publish_image(left_image, self.left_image_publisher)
                self._publish_image(right_image, self.right_image_publisher)
                self.logger.info(f"Publishing image with {self.conversion_type} encoding", once=True)

        except Exception as e:
            self.logger.error(f"An error occurred: {e}")
        finally:
            # Release the camera resource and any other cleanup
            if self.cap.isOpened():
                self.cap.release()
            self.logger.info("Camera released and cleanup done.")

    def destroy_node(self):
        # Release the camera resource when the node is destroyed
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

    def is_valid_video_device(self, device_index):
        """Check if the video device index is valid."""
        if not device_index:
            return True
        cap = cv2.VideoCapture(device_index)
        if not cap.isOpened():
            raise Exception(f"Invalid video device: {device_index}")
        cap.release()
        return True

    def is_valid_resolution(self, width, height):
        """Check if the resolution is valid."""
        # Ensure that width and height are positive values
        if not (width > 0 and height > 0):
            raise Exception(f"Invalid resolution: {width}x{height}")

        return True

    def is_valid_conversion_type(self, conversion_type:str ):
        """Check if the conversion type is valid."""
        if conversion_type == "":
            return True

        if not conversion_type.startswith("COLOR_"):
            raise Exception(f"Invalid conversion type: {conversion_type}")
        
        if not conversion_type.endswith("RGB"):
            raise Exception(f"Can only convert to RGB, current: {conversion_type}")
        
        if not hasattr(cv2, conversion_type):
            raise Exception(f"Invalid conversion type: {conversion_type}")

        return True

def main(args=None):
    rclpy.init(args=args)
    video_publisher = ZedCameraImage()
    video_publisher.capture_and_publish()
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
