"""
Point cloud UDP receiver for MID-360 (port 56301 on host side).

Runs in a background thread, parses incoming 36-byte header + point data,
and stores point clouds in a time-based accumulation buffer.
"""

import collections
import socket
import threading
import time
import logging
from typing import Optional, Tuple

import numpy as np

from . import protocol as proto

logger = logging.getLogger(__name__)

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

        # Time-stamped ring buffer for accumulation
        # Each entry: (timestamp, xyz_array, ref_array)
        self._ring: collections.deque = collections.deque()
        self._accum_sec: float = 0.1  # default accumulation window

        # Stats
        self._frame_count = 0
        self._total_points = 0
        self._last_fps_time = time.time()
        self._fps = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def accum_seconds(self) -> float:
        return self._accum_sec

    @accum_seconds.setter
    def accum_seconds(self, sec: float):
        self._accum_sec = max(0.05, sec)

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
        Get accumulated points within the accumulation window.
        Returns (xyz_Nx3_meters, reflectivity_N) or (None, None).
        """
        with self._lock:
            now = time.time()
            cutoff = now - self._accum_sec
            # Remove old entries
            while self._ring and self._ring[0][0] < cutoff:
                self._ring.popleft()

            if not self._ring:
                return None, None

            xyz_list = [entry[1] for entry in self._ring]
            ref_list = [entry[2] for entry in self._ring]

        xyz = np.concatenate(xyz_list, axis=0)
        ref = np.concatenate(ref_list, axis=0)
        self._total_points = xyz.shape[0]
        return xyz, ref

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def point_count(self) -> int:
        return self._total_points

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

            if pkt.data_type not in (1, 2, 3):
                continue

            xyz, ref = proto.parse_points_numpy(
                pkt.raw_points, pkt.data_type, pkt.dot_num)

            if xyz.shape[0] == 0:
                continue

            now = time.time()
            with self._lock:
                self._ring.append((now, xyz, ref))
                # Prune old entries beyond 2x the accumulation window
                cutoff = now - self._accum_sec * 2
                while self._ring and self._ring[0][0] < cutoff:
                    self._ring.popleft()

            # FPS calculation
            self._frame_count += 1
            dt = now - self._last_fps_time
            if dt >= 1.0:
                self._fps = self._frame_count / dt
                self._frame_count = 0
                self._last_fps_time = now
