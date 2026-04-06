"""
Open3D-based 3D point cloud viewer (non-blocking).

Uses Open3D's VisualizerWithKeyCallback with poll_events/update_renderer
so it can coexist with tkinter's main loop.
"""

import logging
from typing import Optional

import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)


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
        opt.point_size = 1.5

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

    def update(self, xyz: Optional[np.ndarray] = None,
               reflectivity: Optional[np.ndarray] = None) -> bool:
        """
        Update the displayed point cloud and pump Open3D events.
        Call this periodically from the main thread.
        Returns False if the window was closed.
        """
        if not self._vis or not self._running:
            return False

        if xyz is not None and xyz.shape[0] > 0:
            self._pcd.points = o3d.utility.Vector3dVector(xyz)

            # Colorize by height (z-axis) or reflectivity
            if reflectivity is not None and reflectivity.shape[0] == xyz.shape[0]:
                colors = self._colorize_reflectivity(reflectivity)
            else:
                colors = self._colorize_height(xyz)
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
    # Colorization helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _colorize_height(xyz: np.ndarray) -> np.ndarray:
        """Map z-height to a blue-green-red color gradient."""
        z = xyz[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 1e-6:
            return np.full((len(z), 3), 0.5)
        t = (z - z_min) / (z_max - z_min)
        colors = np.zeros((len(z), 3))
        colors[:, 0] = np.clip(t * 2, 0, 1)           # red
        colors[:, 1] = np.clip(1 - abs(t - 0.5) * 2, 0, 1)  # green
        colors[:, 2] = np.clip((1 - t) * 2, 0, 1)     # blue
        return colors

    @staticmethod
    def _colorize_reflectivity(ref: np.ndarray) -> np.ndarray:
        """Map reflectivity (0-255) to a warm color gradient."""
        t = ref.astype(np.float64) / 255.0
        colors = np.zeros((len(t), 3))
        colors[:, 0] = np.clip(t * 1.5, 0, 1)
        colors[:, 1] = np.clip(t * 0.8, 0, 1)
        colors[:, 2] = np.clip(0.3 - t * 0.3, 0, 1)
        return colors
