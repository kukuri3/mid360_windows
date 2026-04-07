"""
Open3D-based 3D point cloud viewer (non-blocking).

Supports two display modes:
  - VIEW: live point cloud only
  - SLAM: KISS-ICP local map + trajectory + current frame
"""

import logging
from enum import Enum
from typing import Optional

import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)


class ColorMode(Enum):
    REFLECTIVITY = "Reflectivity"
    DISTANCE = "Distance"
    HEIGHT_Z = "Height (Z)"


class DisplayMode(Enum):
    VIEW = "View"
    SLAM = "SLAM"


class PointCloudViewer:
    """Non-blocking Open3D point cloud viewer."""

    def __init__(self, window_name: str = "MID-360 Point Cloud"):
        self.window_name = window_name
        self._vis: Optional[o3d.visualization.Visualizer] = None

        # Geometries
        self._pcd = o3d.geometry.PointCloud()           # live frame (VIEW)
        self._map_pcd = o3d.geometry.PointCloud()       # SLAM local map
        self._frame_pcd = o3d.geometry.PointCloud()     # SLAM current frame
        self._trajectory = o3d.geometry.LineSet()       # SLAM trajectory
        self._coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0])

        self._added = {"pcd": False, "map": False, "frame": False, "traj": False}
        self._running = False
        self._point_size = 1.5
        self._color_mode = ColorMode.HEIGHT_Z
        self._display_mode = DisplayMode.VIEW

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(
            window_name=self.window_name, width=1024, height=768)

        self._vis.add_geometry(self._coord_frame)

        opt = self._vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.1])
        opt.point_size = self._point_size

        ctl = self._vis.get_view_control()
        ctl.set_zoom(0.3)
        ctl.set_front([0, -0.5, -1])
        ctl.set_lookat([0, 0, 0])
        ctl.set_up([0, 0, 1])

        self._running = True
        logger.info("Viewer started")

    def stop(self):
        self._running = False
        if self._vis:
            try:
                self._vis.destroy_window()
            except Exception:
                pass
            self._vis = None
        self._added = {k: False for k in self._added}
        logger.info("Viewer stopped")

    @property
    def running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Settings
    # ------------------------------------------------------------------

    @property
    def point_size(self) -> float:
        return self._point_size

    @point_size.setter
    def point_size(self, size: float):
        self._point_size = max(0.5, min(10.0, size))
        if self._vis:
            try:
                self._vis.get_render_option().point_size = self._point_size
            except Exception:
                pass

    @property
    def color_mode(self) -> ColorMode:
        return self._color_mode

    @color_mode.setter
    def color_mode(self, mode: ColorMode):
        self._color_mode = mode

    @property
    def display_mode(self) -> DisplayMode:
        return self._display_mode

    def set_display_mode(self, mode: DisplayMode):
        """Switch between VIEW and SLAM display."""
        if mode == self._display_mode:
            return
        self._display_mode = mode
        # Hide geometries from the inactive mode by clearing their points
        if mode == DisplayMode.VIEW:
            self._clear(self._map_pcd)
            self._clear(self._frame_pcd)
            self._trajectory.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self._trajectory.lines = o3d.utility.Vector2iVector(np.zeros((0, 2), int))
            self._refresh_geometry(self._map_pcd, "map")
            self._refresh_geometry(self._frame_pcd, "frame")
            self._refresh_geometry(self._trajectory, "traj")
        else:
            self._clear(self._pcd)
            self._refresh_geometry(self._pcd, "pcd")

    @staticmethod
    def _clear(pcd: o3d.geometry.PointCloud):
        pcd.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
        pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))

    # ------------------------------------------------------------------
    # Update API — VIEW mode
    # ------------------------------------------------------------------

    def update(self, xyz: Optional[np.ndarray] = None,
               reflectivity: Optional[np.ndarray] = None) -> bool:
        """VIEW mode update + event pump."""
        if not self._vis or not self._running:
            return False

        if (self._display_mode == DisplayMode.VIEW
                and xyz is not None and xyz.shape[0] > 0):
            self._pcd.points = o3d.utility.Vector3dVector(xyz)
            colors = self._colorize(xyz, reflectivity)
            self._pcd.colors = o3d.utility.Vector3dVector(colors)
            self._refresh_geometry(self._pcd, "pcd")

        return self._pump()

    # ------------------------------------------------------------------
    # Update API — SLAM mode
    # ------------------------------------------------------------------

    def update_slam(self,
                    map_points: Optional[np.ndarray],
                    current_frame: Optional[np.ndarray],
                    trajectory: Optional[np.ndarray]) -> bool:
        """SLAM mode update: dim local map + bright current frame + traj line."""
        if not self._vis or not self._running:
            return False

        if self._display_mode != DisplayMode.SLAM:
            return self._pump()

        # Local map (dim)
        if map_points is not None and len(map_points) > 0:
            self._map_pcd.points = o3d.utility.Vector3dVector(map_points)
            map_colors = self._colorize(map_points, None)
            map_colors *= 0.5  # dim
            self._map_pcd.colors = o3d.utility.Vector3dVector(map_colors)
            self._refresh_geometry(self._map_pcd, "map")

        # Current frame (bright)
        if current_frame is not None and len(current_frame) > 0:
            self._frame_pcd.points = o3d.utility.Vector3dVector(current_frame)
            self._frame_pcd.paint_uniform_color([1.0, 1.0, 0.2])  # yellow
            self._refresh_geometry(self._frame_pcd, "frame")

        # Trajectory line
        if trajectory is not None and len(trajectory) >= 2:
            n = len(trajectory)
            lines = np.array([[i, i + 1] for i in range(n - 1)], dtype=np.int32)
            colors = np.tile([1.0, 0.2, 0.2], (len(lines), 1))
            self._trajectory.points = o3d.utility.Vector3dVector(trajectory)
            self._trajectory.lines = o3d.utility.Vector2iVector(lines)
            self._trajectory.colors = o3d.utility.Vector3dVector(colors)
            self._refresh_geometry(self._trajectory, "traj")

        return self._pump()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _refresh_geometry(self, geom, key: str):
        if not self._vis:
            return
        try:
            if not self._added[key]:
                self._vis.add_geometry(geom)
                self._added[key] = True
            else:
                self._vis.update_geometry(geom)
        except Exception:
            pass

    def _pump(self) -> bool:
        try:
            alive = self._vis.poll_events()
            self._vis.update_renderer()
            if not alive:
                self._running = False
                return False
        except Exception:
            self._running = False
            return False
        return True

    # ------------------------------------------------------------------
    # Colorization
    # ------------------------------------------------------------------

    def _colorize(self, xyz: np.ndarray,
                  reflectivity: Optional[np.ndarray]) -> np.ndarray:
        if self._color_mode == ColorMode.REFLECTIVITY:
            if reflectivity is not None and len(reflectivity) == len(xyz):
                return self._colorize_reflectivity(reflectivity)
            return self._colorize_height(xyz)
        elif self._color_mode == ColorMode.DISTANCE:
            return self._colorize_distance(xyz)
        else:
            return self._colorize_height(xyz)

    @staticmethod
    def _colorize_height(xyz: np.ndarray) -> np.ndarray:
        z = xyz[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 1e-6:
            return np.full((len(z), 3), 0.5)
        t = (z - z_min) / (z_max - z_min)
        colors = np.zeros((len(z), 3))
        colors[:, 0] = np.clip(t * 2, 0, 1)
        colors[:, 1] = np.clip(1 - abs(t - 0.5) * 2, 0, 1)
        colors[:, 2] = np.clip((1 - t) * 2, 0, 1)
        return colors

    @staticmethod
    def _colorize_distance(xyz: np.ndarray) -> np.ndarray:
        dist = np.linalg.norm(xyz, axis=1)
        d_min, d_max = dist.min(), dist.max()
        if d_max - d_min < 1e-6:
            return np.full((len(dist), 3), 0.5)
        t = (dist - d_min) / (d_max - d_min)
        colors = np.zeros((len(dist), 3))
        colors[:, 0] = np.clip(t * 2, 0, 1)
        colors[:, 1] = np.clip(1 - abs(t - 0.5) * 2, 0, 1)
        colors[:, 2] = np.clip((1 - t) * 2, 0, 1)
        return colors

    @staticmethod
    def _colorize_reflectivity(ref: np.ndarray) -> np.ndarray:
        t = ref.astype(np.float64) / 255.0
        colors = np.zeros((len(t), 3))
        colors[:, 0] = np.clip(t * 1.5, 0, 1)
        colors[:, 1] = np.clip(t * 0.8, 0, 1)
        colors[:, 2] = np.clip(0.3 - t * 0.3, 0, 1)
        return colors
