"""
SLAM engine wrapping KISS-ICP for the MID-360 demo.

KISS-ICP (https://github.com/PRBonn/kiss-icp) is a simple, robust LiDAR
odometry algorithm with no IMU dependency. We pull scans from the
PointCloudReceiver at a fixed rate, feed them to KISS-ICP, and expose
the resulting pose, trajectory, and local map.
"""

import logging
import threading
import time
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Period at which we pull a new scan and feed it to KISS-ICP
SLAM_SCAN_PERIOD = 0.1   # 100 ms => 10 Hz processing rate
MIN_POINTS_PER_SCAN = 200


class SlamEngine:
    """
    KISS-ICP based LiDAR odometry engine.

    Runs in its own background thread, periodically pulling a scan from
    the receiver and registering it. Thread-safe access to the latest
    pose, trajectory, and local map is provided via get_state().
    """

    def __init__(self, receiver):
        self.receiver = receiver

        self._kiss = None           # KISS-ICP instance
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # Outputs
        self._poses: list = []                       # list of 4x4 SE3
        self._latest_pose: np.ndarray = np.eye(4)
        self._latest_frame_global: Optional[np.ndarray] = None
        self._cursor: float = 0.0                    # last consumed timestamp

        self._running = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> bool:
        """Initialize KISS-ICP and start the processing thread."""
        try:
            from kiss_icp.kiss_icp import KissICP
            from kiss_icp.config import KISSConfig
        except ImportError:
            logger.error("kiss-icp is not installed. Run: pip install kiss-icp")
            return False

        cfg = KISSConfig()
        # MID-360 has ~70m range; use generous bounds
        try:
            cfg.data.max_range = 70.0
            cfg.data.min_range = 0.5
        except Exception:
            pass

        try:
            self._kiss = KissICP(config=cfg)
        except TypeError:
            # Older API
            self._kiss = KissICP(cfg)

        with self._lock:
            self._poses = []
            self._latest_pose = np.eye(4)
            self._latest_frame_global = None
        self._cursor = time.time()

        self._stop.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="slam-engine")
        self._thread.start()
        logger.info("SLAM engine started (KISS-ICP)")
        return True

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
        self._running = False
        self._kiss = None
        logger.info("SLAM engine stopped")

    def reset(self):
        """Clear trajectory and reinitialize KISS-ICP."""
        was_running = self._running
        if was_running:
            self.stop()
        with self._lock:
            self._poses = []
            self._latest_pose = np.eye(4)
            self._latest_frame_global = None
        if was_running:
            self.start()

    def get_state(self) -> dict:
        """Return latest pose, trajectory polyline, and current frame."""
        with self._lock:
            if not self._poses:
                return {
                    "pose": self._latest_pose.copy(),
                    "trajectory": np.zeros((0, 3)),
                    "current_frame": None,
                    "num_poses": 0,
                }
            traj = np.array([p[:3, 3] for p in self._poses])
            return {
                "pose": self._latest_pose.copy(),
                "trajectory": traj,
                "current_frame": (self._latest_frame_global.copy()
                                  if self._latest_frame_global is not None
                                  else None),
                "num_poses": len(self._poses),
            }

    def get_local_map(self) -> Optional[np.ndarray]:
        """Return KISS-ICP's internal voxelized local map (Nx3)."""
        if self._kiss is None:
            return None
        try:
            pts = np.asarray(self._kiss.local_map.point_cloud())
            if pts.size == 0:
                return None
            return pts
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Processing loop
    # ------------------------------------------------------------------

    def _loop(self):
        while not self._stop.is_set():
            self._stop.wait(SLAM_SCAN_PERIOD)
            if self._stop.is_set():
                break

            xyz, ref, latest = self.receiver.get_scan_since(self._cursor)
            self._cursor = latest

            if xyz is None or len(xyz) < MIN_POINTS_PER_SCAN:
                continue

            # Discard zero/near-zero points (invalid measurements)
            r2 = np.einsum('ij,ij->i', xyz, xyz)
            mask = r2 > 0.25  # > 0.5m
            if mask.sum() < MIN_POINTS_PER_SCAN:
                continue
            xyz_clean = xyz[mask]

            try:
                self._register_frame(xyz_clean)
            except Exception:
                logger.exception("KISS-ICP register_frame failed")
                continue

    def _register_frame(self, xyz: np.ndarray):
        """
        Call KISS-ICP register_frame across multiple API versions and
        update internal state with the resulting pose.
        """
        timestamps = np.zeros(len(xyz), dtype=np.float64)

        # Try the modern API first
        try:
            self._kiss.register_frame(xyz, timestamps)
        except TypeError:
            try:
                self._kiss.register_frame(frame=xyz, timestamps=timestamps)
            except TypeError:
                self._kiss.register_frame(xyz)

        try:
            pose = np.asarray(self._kiss.last_pose)
        except Exception:
            poses = getattr(self._kiss, "poses", None)
            if poses is None or len(poses) == 0:
                return
            pose = np.asarray(poses[-1])

        if pose.shape != (4, 4):
            return

        # Transform current scan into global frame
        homog = np.hstack([xyz, np.ones((len(xyz), 1))])
        global_pts = (pose @ homog.T).T[:, :3]

        with self._lock:
            self._poses.append(pose.copy())
            self._latest_pose = pose.copy()
            self._latest_frame_global = global_pts
