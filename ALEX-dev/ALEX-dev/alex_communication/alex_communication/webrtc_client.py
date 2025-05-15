import logging
import asyncio
import socketio
import argparse
from pathlib import Path
from yaml import safe_load
from aiortc import RTCPeerConnection, RTCSessionDescription
from alex_utilities.urp_auth_utilities import get_encoded_room_id
from alex_externals.camera_handler import ROSCameraStreamTrack, WebcamVideoStreamTrack

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebRTCClient:
    def __init__(self, access_token: str, host:str, source_type:str="webcam", **kwargs):
        if not access_token:
            raise KeyError("Could kindly pass the access token")
        if not host:
            raise KeyError("Could kindly pass the urp host url")

        logger.info(f"Got source type as : {source_type}")
        if source_type == "webcam":
            self.StreamTrack = lambda : WebcamVideoStreamTrack(kwargs["webcam_index"])
        elif source_type == "ros":
            self.StreamTrack = lambda : ROSCameraStreamTrack(kwargs["ros_camera_topics"])
        else:
            raise KeyError("Got undefined Source Type")

        self.urp_host:str = host
        self.pcs:dict[str, RTCPeerConnection] = {}
        self.room = get_encoded_room_id(access_token)
        self.sio = socketio.AsyncClient()
        
        self._register_events()
    
    def _register_events(self):
        self.sio.on("connect", self.connect)
        self.sio.on("offer", self.offer)
        # self.sio.on("force_close", self.close_all)
    
    async def connect(self,):
        logger.info('Connected to signaling server')
        await self.sio.emit('join_room', self.room)
    
    async def offer(self, msg):
        logger.info('offer received')
        uuid = msg["uuid"]
        data = msg["data"]

        pc = RTCPeerConnection()
        self.pcs[uuid] = pc
        pc.addTrack(self.StreamTrack())

        await pc.setRemoteDescription(RTCSessionDescription(data['sdp'], data['type']))

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        @pc.on("iceconnectionstatechange")
        async def on_connectionstatechange():
            print("iceConnectionState state is %s" % pc.iceConnectionState)
            if pc.iceConnectionState in ["failed", "disconnected", "closed"]:
                self.pcs.pop(uuid)
                await pc.close()

        await self.sio.emit('video_feed', {
            'topic': 'answer',
            'msg': {
                "data": {'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type},
                "uuid": uuid
                }, 
            'roomId': self.room
        })
        logger.info(f"answer send to {uuid=}")
    
    async def close_all(self):
        for uuid, pc in self.pcs.items():
            print(f"Closing connection with {uuid=}")
            await pc.close()
        print("All peer connection closed!!")

    async def run(self):
        await self.sio.connect(self.urp_host)
        await self.sio.wait()

def handle_ros_args(args, node_name = "webrtc_client"):
    params_path = Path(args.params_file)
    if not params_path.exists():
        raise FileNotFoundError(f"file does exist {args.params_file}")

    if not params_path.is_file():
        raise FileNotFoundError(f"path is not a file {args.params_file}")
    
    if not params_path.suffix == ".yaml":
        raise FileNotFoundError(f"path is not a yaml file {args.params_file}")
    
    remap_args = args.remap
    remap = remap_args and {item.split(":=")[0]:item.split(":=")[1] for item in remap_args} or {}

    node_name = "/" + remap.get("__node", node_name)

    params_path.read_text()
    params_data = safe_load(params_path.open())

    parameters = {}

    if "/**" in params_data:
        parameters.update(params_data["/**"]["ros__parameters"])
    if node_name in params_data:
        parameters.update(params_data[node_name]["ros__parameters"])

    return parameters

def parse_args():
    parser = argparse.ArgumentParser(description="WebRTC Client")
    parser.add_argument(
        '--ros-args', 
        action='store_true', 
        help="[ROS] Parse arguments from ROS params"
    )
    parser.add_argument(
        '-r', '--remap', 
        action='append', 
        help="[ROS] Remap arguments in key:=value format"
    )
    parser.add_argument(
        '--params-file', 
        help="[ROS] Path to the parameters file"
    )

    parser.add_argument(
        '--host',
        type=str,
        help="[Terminal] URP host address"
    )

    parser.add_argument(
        '--access-token', 
        type=str,
        help="[Terminal] URP Access token"
    )
    parser.add_argument(
        '--source-type', 
        type=str,
        default="webcam",
        choices=["webcam", "ros"],
        help="[Terminal] Camera source type"
    )

    parser.add_argument(
        "--webcam-index",
        default=0,
        help="[Terminal] web camera port or index while using souce type 'webcam'"
    )

    parser.add_argument(
        "--ros-camera-topics",
        help="[Terminal] topic to echo while using source type 'ros'",
        nargs='+',
    )
    
    args = parser.parse_args()

    keyword_arguments = vars(args) if not args.ros_args else handle_ros_args(args)

    return keyword_arguments

def main():
    keyword_arguments = parse_args()

    client = WebRTCClient(**keyword_arguments)
    asyncio.run(client.run())

if __name__ == "__main__":
    main()
