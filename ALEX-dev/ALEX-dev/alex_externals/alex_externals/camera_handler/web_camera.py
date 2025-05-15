import cv2
import time
import asyncio
from typing import Tuple
from av import VideoFrame
from fractions import Fraction
from aiortc.mediastreams import MediaStreamError, MediaStreamTrack
from alex_utilities import create_text_image

VIDEO_PTIME = 1 / 30  # Assuming 30 fps
VIDEO_CLOCK_RATE = 90000  # Standard WebRTC clock rate for video
VIDEO_TIME_BASE = Fraction(1, VIDEO_CLOCK_RATE)

class WebcamManager:
    _instance = None
    _connection_count = 0

    @staticmethod
    def get_instance():
        if WebcamManager._instance is None:
            WebcamManager()
        return WebcamManager._instance

    def __init__(self):
        if WebcamManager._instance is not None:
            raise Exception("This class is a singleton!")
        WebcamManager._instance = self
        self.cap = None

    def start_camera(self, webcam_index):
        if self.cap is None:
            self.cap = cv2.VideoCapture(webcam_index)
        WebcamManager._connection_count += 1
        print(f"Camera started. Active connections: {WebcamManager._connection_count}")

    def stop_camera(self):
        WebcamManager._connection_count -= 1
        print(f"Stopping camera. Active connections: {WebcamManager._connection_count}")
        if WebcamManager._connection_count <= 0:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
                print("Webcam stopped")

    def get_frames(self):
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None
    
class WebcamVideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, webcam_index=0):
        super().__init__()
        self.camera_manager = WebcamManager.get_instance()
        self.camera_manager.start_camera(webcam_index)  # Start the camera when this track is initialized
        self._start = time.time()
        self.webcam_index = webcam_index
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
        frame = self.camera_manager.get_frames()
        if frame is None:
            print("No frame received from webcam.")
            frame = create_text_image("Waiting for", "webcam image")
        
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(rgb_image, format='rgb24')
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        super().stop()
        self.camera_manager.stop_camera()
    
