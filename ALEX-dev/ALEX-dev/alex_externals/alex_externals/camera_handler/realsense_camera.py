import cv2
import time
import numpy
import asyncio
import pyrealsense2
from typing import Tuple
from av import VideoFrame
from fractions import Fraction
from aiortc.mediastreams import MediaStreamError, MediaStreamTrack

VIDEO_PTIME = 1 / 30  # Assuming 30 fps
VIDEO_CLOCK_RATE = 90000  # Standard WebRTC clock rate for video
VIDEO_TIME_BASE = Fraction(1, VIDEO_CLOCK_RATE)

class RealSenseCameraManager:
    _instance = None
    _connection_count = 0

    @staticmethod
    def get_instance():
        if RealSenseCameraManager._instance is None:
            RealSenseCameraManager()
        return RealSenseCameraManager._instance

    def __init__(self):
        if RealSenseCameraManager._instance is not None:
            raise Exception("This class is a singleton!")
        RealSenseCameraManager._instance = self
        self.pipeline = None

    def start_camera(self):
        if self.pipeline is None:
            self.pipeline = pyrealsense2.pipeline()
            config = pyrealsense2.config()
            config.enable_stream(pyrealsense2.stream.color, 640, 480, pyrealsense2.format.bgr8, 30)
            self.pipeline.start(config)
        RealSenseCameraManager._connection_count += 1
        print(f"Camera started.  Active connections: {RealSenseCameraManager._connection_count}")

    def stop_camera(self):
        RealSenseCameraManager._connection_count -= 1
        print(f"Stopping camera. Active connections: {RealSenseCameraManager._connection_count}")
        if RealSenseCameraManager._connection_count <= 0:
            if self.pipeline is not None:
                self.pipeline.stop()
                self.pipeline = None
                print("Camera stopped")

    def get_frames(self):
        if self.pipeline:
            return self.pipeline.wait_for_frames()
        return None
    
class RealSenseVideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.camera_manager = RealSenseCameraManager.get_instance()
        self.camera_manager.start_camera()  # Start the camera when this track is initialized
        self._start = time.time()
        self._timestamp = 0

    async def next_timestamp(self) -> Tuple[int, Fraction]:
        if self.readyState != "live":
            raise MediaStreamError
        self._timestamp += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
        wait = self._start + (self._timestamp / VIDEO_CLOCK_RATE) - time.time()
        await asyncio.sleep(wait)
        return self._timestamp, VIDEO_TIME_BASE

    async def recv(self) -> VideoFrame:
        pts, time_base = await self.next_timestamp()
        frames = self.camera_manager.get_frames()
        if not frames:
            raise MediaStreamError("No frame received from RealSense camera")
        color_frame = frames.get_color_frame()
        color_image = numpy.asanyarray(color_frame.get_data())
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(rgb_image, format='rgb24')
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        super().stop()
        self.camera_manager.stop_camera()
    
