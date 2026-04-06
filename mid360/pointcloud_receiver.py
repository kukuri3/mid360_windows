"""
Point cloud UDP receiver for MID-360 (port 56301 on host side).

Runs in a background thread, parses incoming 36-byte header + point data,
and stores the latest point cloud in a thread-safe buffer.
"""

import socket
import threading
import time
import logging
from typing import Optional, Tuple

import numpy as np

from . import protocol as proto

logger = logging.getLogger(__name__)

# Maximum UDP datagram size
RECV_BUF_SIZE = 65535


class PointCloudReceiver:
    """Receives and buffers point cloud data from MID-360."""

    def __init__(self, host_ip: str, port: int = proto.PORT_HOST_POINTCLOUD):
        self.host_ip = host_ip
        self.port = port

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # Accumulation buffer — collects points between get_points() calls
        self._accum_points: list = []
        self._accum_ref: list = []

        # Stats
        self._frame_count = 0
        self._point_count = 0
        self._last_fps_time = time.time()
        self._fps = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Start listening for point cloud data."""
        self._stop_event.clear()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)
        self._sock.bind((self.host_ip, self.port))
        self._sock.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)

        self._thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="pcl-receiver")
        self._thread.start()
        logger.info("Point cloud receiver started on %s:%d",
                     self.host_ip, self.port)

    def stop(self):
        """Stop the receiver."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        logger.info("Point cloud receiver stopped")

    def get_points(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get accumulated points since last call.
        Returns (xyz_Nx3_meters, reflectivity_N) or (None, None).
        Clears the accumulation buffer.
        """
        with self._lock:
            if not self._accum_points:
                return None, None
            xyz = np.concatenate(self._accum_points, axis=0)
            ref = np.concatenate(self._accum_ref, axis=0)
            self._accum_points.clear()
            self._accum_ref.clear()
        return xyz, ref

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def point_count(self) -> int:
        return self._point_count

    # ------------------------------------------------------------------
    # Receive loop
    # ------------------------------------------------------------------

    def _recv_loop(self):
        while not self._stop_event.is_set():
            try:
                data, addr = self._sock.recvfrom(RECV_BUF_SIZE)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    break
                continue

            pkt = proto.parse_pointcloud_packet(data)
            if pkt is None:
                continue

            # Parse points based on data_type
            if pkt.data_type not in (1, 2, 3):
                continue

            xyz, ref = proto.parse_points_numpy(
                pkt.raw_points, pkt.data_type, pkt.dot_num)

            if xyz.shape[0] == 0:
                continue

            with self._lock:
                self._accum_points.append(xyz)
                self._accum_ref.append(ref)
                self._point_count = xyz.shape[0]

            # FPS calculation
            self._frame_count += 1
            now = time.time()
            dt = now - self._last_fps_time
            if dt >= 1.0:
                self._fps = self._frame_count / dt
                self._frame_count = 0
                self._last_fps_time = now
