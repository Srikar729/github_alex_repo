import cv2
import time
import rclpy
import logging
import asyncio
import numpy as np
from av import VideoFrame
from typing import Callable
from rclpy.node import Node
from fractions import Fraction
from sensor_msgs.msg import Image
from alex_interfaces.srv import SwitchCamera
from aiortc.mediastreams import MediaStreamTrack, MediaStreamError
from alex_utilities import create_text_image, merge_frames


VIDEO_PTIME = 1 / 30  # Assuming 30 fps
VIDEO_CLOCK_RATE = 90000  # Standard WebRTC clock rate for video
VIDEO_TIME_BASE = Fraction(1, VIDEO_CLOCK_RATE)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROSCameraSubscriber(Node):
    _instance = None
    _reference_count = 0
    _instance_history: set[Callable] = set()

    def __init__(self, topic_names: list[str]):
        super().__init__('ros_camera_subscriber')
        self.latest_frame:dict[int] = {}
        for num_id, topic_name in enumerate(topic_names, 1):
            self.latest_frame[num_id] = None
            self.create_subscription(Image, topic_name, self.image_subscriber(num_id), 1)

        self.create_service(SwitchCamera, "switch_camera", self.switch_camera)
        logger.info(f"Subscribed to ROS2 topic: {topic_names}")
    
    def image_subscriber(self, index):
        # Wrapper function to allow sending data to function with index
        def callback(data):
            return self.image_callback(index, data)
        return callback

    def image_callback(self, index, msg: Image):
        # Convert ROS Image message to OpenCV image
        frame = self.ros_image_to_opencv(msg)
        # Convert ROS Image message to OpenCV image
        frame = self.ros_image_to_opencv(msg)
        if msg.encoding == "bgra8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        elif msg.encoding == "bgr8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.latest_frame[index] = frame
        logger.debug("Image received and updated.")
    
    def switch_camera(self, request: SwitchCamera.Request, response: SwitchCamera.Response):
        for function in self._instance_history:
            result = function(request.index)
        return response

    @staticmethod
    def ros_image_to_opencv(img_msg: Image):
        height = img_msg.height
        width = img_msg.width
        img_data = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, width, -1)
        return img_data

    @classmethod
    def get_instance(cls, topic_name: str):
        if cls._instance is None:
            cls._instance = cls(topic_name)
            rclpy.get_global_executor().add_node(cls._instance)
        return cls._instance

    @classmethod
    def shutdown_node(cls):
        logger.info("Shutting down ROS2 node.")
        if cls._instance:
            cls._instance.destroy_node()
            rclpy.shutdown()
            cls._instance = None
        if cls._instance_history:
            cls._instance_history.clear()

    @classmethod
    def increase_reference(cls, call_func: Callable):
        cls._reference_count += 1
        cls._instance_history.add(call_func)
        logger.info(f"New client connected. Active connections: {cls._reference_count}")

    @classmethod
    def decrease_reference(cls, call_func: Callable):
        cls._reference_count -= 1
        logger.info(f"Client disconnected. Active connections: {cls._reference_count}")
        if call_func in cls._instance_history:
            cls._instance_history.remove(call_func)
        if cls._reference_count <= 0:
            logger.info("No more active connections. Initiating node shutdown.")
            rclpy.get_global_executor().remove_node(cls._instance)
            cls.shutdown_node()

class ROSCameraStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, topic_names: list[str]):
        super().__init__()  # Initialize MediaStreamTrack
        self._start = time.time()
        self._timestamp = 0
        self._image_index = 1
        self.topic_names = topic_names

        # Ensure ROS2 node and subscriber are initialized only once
        if not rclpy.ok():
            logger.info("Initializing ROS2 and starting the ROS2 node.")
            rclpy.init()
            self.node = ROSCameraSubscriber.get_instance(topic_names)
            self.loop = asyncio.get_event_loop()
            self.loop.run_in_executor(None, rclpy.get_global_executor().spin)

        # Increase reference count when a new track is created
        ROSCameraSubscriber.increase_reference(self.switch_camera)
    
    def get_latest_frame(self,):
        images = ROSCameraSubscriber.get_instance(None).latest_frame
        if not any( image is not None for image in images.values()):
            return None

        if self._image_index == 0:
            frames = [ image for image in images.values() if image is not None ]
            frame = merge_frames(frames)
            return frame
        
        frame = images.get(self._image_index)
        return frame        

    async def next_timestamp(self):
        if self.readyState != "live":
            raise MediaStreamError
        self._timestamp += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
        wait = self._start + (self._timestamp / VIDEO_CLOCK_RATE) - time.time()
        await asyncio.sleep(wait)
        return self._timestamp, VIDEO_TIME_BASE

    async def recv(self) -> VideoFrame:
        pts, time_base = await self.next_timestamp()

        frame = self.get_latest_frame()
        if frame is None:
            # logger.error("No frame received from ROS topic.")
            frame = create_text_image("Waiting for", "ROS image")

        video_frame = VideoFrame.from_ndarray(frame)
        video_frame.pts = pts
        video_frame.time_base = time_base
        logger.debug("Frame sent to WebRTC client.")
        return video_frame

    def stop(self):
        super().stop()
        # Decrease reference count when a track is stopped
        ROSCameraSubscriber.decrease_reference()
    
    def switch_camera(self, camera_number:int):
        if camera_number < 0: return 
        if camera_number > len(self.topic_names): return
        self._image_index = camera_number
        return True
