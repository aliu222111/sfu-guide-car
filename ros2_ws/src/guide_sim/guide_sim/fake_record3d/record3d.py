"""Stand-in for the ``record3d`` library: the simulated iPhone instead of a USB one.

The hardware-in-the-loop launch puts this directory first on the iPhone
driver's PYTHONPATH, so the real driver code runs unchanged. Frames come from
the simulator (``guide_sim.devices.Record3DServer``) over the UNIX socket named
in R3D_SIM_SOCKET. Same API as the library's Python bindings.
"""

import os
import pickle
import socket
import struct
import threading
from types import SimpleNamespace

SOCKET = os.environ.get("R3D_SIM_SOCKET", "/tmp/guide_sim/record3d.sock")


class Record3DStream:
    @staticmethod
    def get_connected_devices():
        return [SimpleNamespace(product_id=0, udid="guide-sim")] if os.path.exists(SOCKET) else []

    def __init__(self):
        self.on_new_frame = None
        self.on_stream_stopped = None
        self.frame = None

    def connect(self, dev):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(SOCKET)
        threading.Thread(target=self._read, args=(s,), daemon=True).start()
        return True

    def _read(self, s):
        f = s.makefile("rb")
        try:
            while True:
                head = f.read(4)
                if len(head) < 4:
                    break
                body = f.read(struct.unpack("<I", head)[0])
                self.frame = pickle.loads(body)
                if self.on_new_frame:
                    self.on_new_frame()
        finally:
            if self.on_stream_stopped:
                self.on_stream_stopped()

    def get_rgb_frame(self):
        return self.frame["rgb"]

    def get_depth_frame(self):
        return self.frame["depth"]

    def get_confidence_frame(self):
        return self.frame["conf"]

    def get_intrinsic_mat(self):
        fx, fy, tx, ty = self.frame["K"]
        return SimpleNamespace(fx=fx, fy=fy, tx=tx, ty=ty)

    def get_camera_pose(self):
        qx, qy, qz, qw, tx, ty, tz = self.frame["pose"]
        return SimpleNamespace(qx=qx, qy=qy, qz=qz, qw=qw, tx=tx, ty=ty, tz=tz)

    def get_device_type(self):
        return 1  # LiDAR
