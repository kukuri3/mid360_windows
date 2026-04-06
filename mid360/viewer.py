"""
Open3D-based 3D point cloud viewer (non-blocking).

Uses Open3D Visualizer with poll_events/update_renderer
so it can coexist with tkinter's main loop.
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


class PointCloudViewer:
    """Non-blocking Open3D point cloud viewer."""

    def __init__(self, window_name: str = "MID-360 Point Cloud"):
        self.window_name = window_name
        self._vis: Optional[o3d.visualization.Visualizer] = None
        self._pcd = o3d.geometry.PointCloud()
        self._coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0])
        self._geometry_added = False
        self._running = False
        self._point_size = 1.5
        self._color_mode = ColorMode.HEIGHT_Z

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Create and show the Open3D window."""
        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(
            window_name=self.window_name, width=1024, height=768)

        # Add coordinate axes
        self._vis.add_geometry(self._coord_frame)

        # Set render options
        opt = self._vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.1])
        opt.point_size = self._point_size

        # Set initial viewpoint
        ctl = self._vis.get_view_control()
        ctl.set_zoom(0.3)
        ctl.set_front([0, -0.5, -1])
        ctl.set_lookat([0, 0, 0])
        ctl.set_up([0, 0, 1])

        self._running = True
        logger.info("Viewer started")

    def stop(self):
        """Destroy the Open3D window."""
        self._running = False
        if self._vis:
            try:
                self._vis.destroy_window()
            except Exception:
                pass
            self._vis = None
        self._geometry_added = False
        logger.info("Viewer stopped")

    @property
    def running(self) -> bool:
        return self._running

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

    def update(self, xyz: Optional[np.ndarray] = None,
               reflectivity: Optional[np.ndarray] = None) -> bool:
        """
        Update the displayed point cloud and pump Open3D events.
        Returns False if the window was closed.
        """
        if not self._vis or not self._running:
            return False

        if xyz is not None and xyz.shape[0] > 0:
            self._pcd.points = o3d.utility.Vector3dVector(xyz)
            colors = self._colorize(xyz, reflectivity)
            self._pcd.colors = o3d.utility.Vector3dVector(colors)

            if not self._geometry_added:
                self._vis.add_geometry(self._pcd)
                self._geometry_added = True
            else:
                self._vis.update_geometry(self._pcd)

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
