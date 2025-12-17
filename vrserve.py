# vrserve.py – Low-latency stereo streamer with HUD overlay (WebRTC + MJPEG)
import asyncio, fractions, os, string, secrets, struct, json, time
from concurrent.futures import ThreadPoolExecutor
import cv2, numpy as np
from av import VideoFrame
from aiortc import (
    RTCPeerConnection, RTCSessionDescription, VideoStreamTrack,
    RTCConfiguration, RTCIceServer
)
from datetime import datetime

import firebase_admin
from firebase_admin import credentials, firestore

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

# ============================ CONFIGURATION =================================
CAM_WIDTH   = 1280
CAM_HEIGHT  = 720
CROP_TOP    = 40
CROP_BOT    = 40
OUT_HEIGHT  = CAM_HEIGHT - CROP_TOP - CROP_BOT  # 640
CAM_FPS     = 30
CAM_FOURCC  = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')  # MJPEG
CAM_DEVICE  = 0

DEVICENAME       = "/dev/ttyACM0"
BAUDRATE         = 1_000_000
PROTOCOL_VERSION = 2.0
MOTOR_IDS        = [10, 11, 12]
ADDR_TORQUE_ENABLE  = 64
ADDR_GOAL_POSITION  = 116
LEN_GOAL_POSITION   = 4
TORQUE_ENABLE       = 1
TORQUE_DISABLE      = 0

SIGNAL_KEY_JSON = "/home/kg/orbital-rtc/signal_key.json"

REMOTE_SDP: str | None = None

# ============================ UTILS ==========================================

def gen_room_code(length: int = 4) -> str:
    chars = string.ascii_uppercase + string.digits
    return ''.join(secrets.choice(chars) for _ in range(length))

def _answer_cb(snap, *_):
    global REMOTE_SDP
    for doc in snap:
        data = doc.to_dict()
        if data and 'answer' in data:
            REMOTE_SDP = data['answer']

# ============================ CAMERA TRACK ===================================
class MJPEGCroppedTrack(VideoStreamTrack):
    """MJPEG 1280×720 capture → crop to 1280×640, output as bgr24 with HUD."""
    def __init__(self, cam_id: int = 0):
        super().__init__()
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, CAM_FOURCC)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   2)
        if not self.cap.isOpened():
            raise RuntimeError("USB camera open failed")
        self.time_base = fractions.Fraction(1, CAM_FPS)
        self.pts = 0
        self.hud_path = "/home/kg/orbital-rtc/hud_data.json"
        self._hud_cache = {}
        self._hud_last_read = 0.0

    def _read_hud_data(self):
        now = time.time()
        if now - self._hud_last_read < 1.0:
            return self._hud_cache
        try:
            with open(self.hud_path, "r") as f:
                self._hud_cache = json.load(f)
        except:
            self._hud_cache = {}
        self._hud_last_read = now
        return self._hud_cache

    def _draw_hud(self, frame):
        hud = self._read_hud_data()
        if not hud:
            return frame

        overlay = frame.copy()
        box_w, box_h = 200, 25 * len(hud) + 10
        alpha = 0.6
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        color = (0, 255, 0)
        thickness = 1

        # Define positions for both eyes
        eyes = {
            "left": (220, 80),
            "right": (640 + 180, 70)
        }

        for eye, (x, y_start) in eyes.items():

            y = y_start
            for key, val in hud.items():
                text = f"{key}: {val}"
                cv2.putText(overlay, text, (x + 10, y), font, scale, (0, 0, 0), 2, cv2.LINE_AA)  # Shadow
                cv2.putText(overlay, text, (x + 10, y), font, scale, color, thickness, cv2.LINE_AA)
                y += 25

        # Blend overlay
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
        return frame

    async def recv(self):
        loop = asyncio.get_event_loop()
        ret, frame = await loop.run_in_executor(None, self.cap.read)
        if not ret:
            raise RuntimeError("Camera read failed")
        frame = frame[CROP_TOP:CAM_HEIGHT - CROP_BOT, :]
        frame = self._draw_hud(frame)
        self.pts += 1
        vf = VideoFrame.from_ndarray(frame, format="bgr24")
        vf.pts = self.pts
        vf.time_base = self.time_base
        return vf

    def __del__(self):
        if getattr(self, 'cap', None) and self.cap.isOpened():
            self.cap.release()

# ============================ DYNAMIXEL CTRL =================================
class DynamixelController:
    TICKS_PER_DEG = 4096.0 / 360.0
    MID = 2048

    def __init__(self):
        self.port = PortHandler(DEVICENAME)
        if not self.port.openPort():
            raise RuntimeError("DXL port open failed")
        self.port.setBaudRate(BAUDRATE)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        ADDR_D, ADDR_I, ADDR_P = 80, 82, 84
        PID_D, PID_I, PID_P = 10, 5, 1200
        for m in MOTOR_IDS:
            self.packet.write2ByteTxRx(self.port, m, ADDR_D, PID_D)
            self.packet.write2ByteTxRx(self.port, m, ADDR_I, PID_I)
            self.packet.write2ByteTxRx(self.port, m, ADDR_P, PID_P)
            self.packet.write1ByteTxRx(self.port, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.group = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    def set_positions(self, ang_deg: np.ndarray):
        self.group.clearParam()
        for m, d in zip(MOTOR_IDS, ang_deg):
            ticks = int(d * self.TICKS_PER_DEG) + self.MID
            ticks = max(0, min(4095, ticks))
            self.group.addParam(m, struct.pack('<I', ticks))
        self.group.txPacket()

    def disable(self):
        for m in MOTOR_IDS:
            self.packet.write1ByteTxRx(self.port, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.port.closePort()

# ============================ FIRESTORE ICE ==================================
async def publish_local_ice(pc, room):
    ref = firestore.client().collection('rooms').document(room).collection('callerCandidates')
    @pc.on('icecandidate')
    async def _ice(e):
        if e.candidate:
            ref.add({'candidate': e.candidate.to_sdp()})

def subscribe_remote_ice(pc, room):
    ref = firestore.client().collection('rooms').document(room).collection('calleeCandidates')
    def _cb(snap, *_):
        for doc in snap:
            cand = doc.to_dict().get('candidate')
            if cand:
                pc.addIceCandidate(cand); doc.reference.delete()
    ref.on_snapshot(_cb)

# ============================ SESSION ========================================
async def run_session(pc, channel, room, dxl, exe):
    @channel.on('message')
    async def _msg(msg):
        arr = np.frombuffer(msg, dtype=np.float32)
        arr = np.clip(arr, [-120, -80, -55], [120, 80, 55])
        dxl.set_positions(arr)
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"[WEBRTC] Connection state is {pc.connectionState}")

    offer = await pc.createOffer(); await pc.setLocalDescription(offer)
    room_ref = firestore.client().collection('rooms').document(room)
    room_ref.update({'offer': pc.localDescription.sdp})
    room_ref.on_snapshot(_answer_cb)
    print('[WEBRTC] Offer sent')

    while REMOTE_SDP is None:
        await asyncio.sleep(0.01)
    await pc.setRemoteDescription(RTCSessionDescription(REMOTE_SDP, 'answer'))
    print('[WEBRTC] Answer set, streaming…')

    while pc.connectionState not in ('closed', 'failed', 'disconnected'):
        await asyncio.sleep(0.1)
    
    await pc.close()

# ============================ MAIN LOOP ======================================
async def main():
    global REMOTE_SDP
    os.nice(-10)
    firebase_admin.initialize_app(credentials.Certificate(SIGNAL_KEY_JSON))

    room = gen_room_code()
    print(f'[ROOM] Code (permanent) = {room}')

    cam_track = MJPEGCroppedTrack()
    dxl = DynamixelController()
    exe = ThreadPoolExecutor(max_workers=2)

    while True:
        REMOTE_SDP = None
        room_ref = firestore.client().collection('rooms').document(room)
        room_ref.set({'offer': None, 'answer': None})

        for sub in ('callerCandidates', 'calleeCandidates'):
            for doc in room_ref.collection(sub).list_documents():
                doc.delete()

        pc = RTCPeerConnection(RTCConfiguration(iceServers=[
            RTCIceServer(urls=['stun:stun.l.google.com:19302'])
        ]))
        subscribe_remote_ice(pc, room)
        await publish_local_ice(pc, room)
        pc.addTrack(cam_track)
        dc = pc.createDataChannel('ctrl')

        try:
            await run_session(pc, dc, room, dxl, exe)
        except:
            print("[WEBRTC] Shutting down server")
            room_ref.delete()
            dxl.disable()
            exe.shutdown()
            return

if __name__ == '__main__':
    asyncio.run(main())

