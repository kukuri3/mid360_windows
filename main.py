"""
MID-360 Point Cloud Demo — main entry point.

Launches the tkinter control panel and Open3D viewer,
wiring them together with the MID-360 connection, receiver,
and KISS-ICP SLAM engine.
"""

import logging
import os
import time
from tkinter import filedialog, messagebox

import numpy as np

from mid360.connection import MID360Connection
from mid360.pointcloud_receiver import PointCloudReceiver
from mid360.viewer import PointCloudViewer, ColorMode, DisplayMode
from mid360.slam_engine import SlamEngine
from gui.control_panel import ControlPanel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

VIEWER_UPDATE_MS = 33  # ~30 fps

_COLOR_MODE_MAP = {
    "Height (Z)": ColorMode.HEIGHT_Z,
    "Reflectivity": ColorMode.REFLECTIVITY,
    "Distance": ColorMode.DISTANCE,
}


class App:
    def __init__(self):
        self._conn = None
        self._receiver = None
        self._viewer = None
        self._slam = None
        self._mode = "View"  # "View" or "SLAM"

        self._panel = ControlPanel(
            on_connect=self._handle_connect,
            on_disconnect=self._handle_disconnect,
            on_accum_changed=self._handle_accum_changed,
            on_point_size_changed=self._handle_point_size_changed,
            on_color_mode_changed=self._handle_color_mode_changed,
            on_mode_changed=self._handle_mode_changed,
            on_save_map=self._handle_save_map,
            on_reset_slam=self._handle_reset_slam,
        )

        self._panel.schedule(VIEWER_UPDATE_MS, self._update_loop)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _handle_connect(self, sensor_ip: str, host_ip: str):
        try:
            conn = MID360Connection(sensor_ip, host_ip)
            if not conn.connect():
                self._panel.root.after(
                    0, lambda: self._panel.set_connected(False))
                logger.error("Failed to connect to %s", sensor_ip)
                return

            receiver = PointCloudReceiver(host_ip)
            receiver.accum_seconds = 0.5
            receiver.start()

            if not conn.start_sampling():
                logger.warning("start_sampling command failed")

            self._conn = conn
            self._receiver = receiver

            self._panel.root.after(0, self._start_viewer)
            self._panel.root.after(0, lambda: self._panel.set_connected(True))
            # If user already had SLAM mode selected, start the engine
            self._panel.root.after(
                100, lambda: self._handle_mode_changed(self._panel.current_mode))

        except Exception:
            logger.exception("Connection error")
            self._panel.root.after(0, lambda: self._panel.set_connected(False))

    def _handle_disconnect(self):
        if self._slam:
            self._slam.stop()
            self._slam = None

        if self._conn:
            try:
                self._conn.stop_sampling()
            except Exception:
                pass
            self._conn.disconnect()
            self._conn = None

        if self._receiver:
            self._receiver.stop()
            self._receiver = None

        if self._viewer:
            self._viewer.stop()
            self._viewer = None

        if not self._panel.closing:
            self._panel.set_connected(False)

    # ------------------------------------------------------------------
    # Display callbacks
    # ------------------------------------------------------------------

    def _handle_accum_changed(self, seconds: float):
        if self._receiver:
            self._receiver.accum_seconds = seconds

    def _handle_point_size_changed(self, size: float):
        if self._viewer:
            self._viewer.point_size = size

    def _handle_color_mode_changed(self, mode_label: str):
        if self._viewer:
            self._viewer.color_mode = _COLOR_MODE_MAP.get(
                mode_label, ColorMode.HEIGHT_Z)

    # ------------------------------------------------------------------
    # SLAM mode
    # ------------------------------------------------------------------

    def _handle_mode_changed(self, mode: str):
        self._mode = mode
        if self._viewer:
            self._viewer.set_display_mode(
                DisplayMode.SLAM if mode == "SLAM" else DisplayMode.VIEW)

        if mode == "SLAM":
            if self._receiver and not self._slam:
                self._slam = SlamEngine(self._receiver)
                if not self._slam.start():
                    self._slam = None
                    messagebox.showerror(
                        "SLAM Error",
                        "Failed to start KISS-ICP. Run:\n  pip install kiss-icp")
                    self._panel._mode_var.set("View")
                    self._panel._refresh_slam_buttons()
                    if self._viewer:
                        self._viewer.set_display_mode(DisplayMode.VIEW)
        else:
            if self._slam:
                self._slam.stop()
                self._slam = None

    def _handle_save_map(self):
        if not self._slam:
            return
        path = filedialog.asksaveasfilename(
            title="Save SLAM map",
            defaultextension=".ply",
            filetypes=[("PLY point cloud", "*.ply"),
                       ("PCD point cloud", "*.pcd")])
        if not path:
            return
        try:
            pts = self._slam.get_local_map()
            if pts is None or len(pts) == 0:
                messagebox.showwarning("Save Map", "Map is empty")
                return
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            o3d.io.write_point_cloud(path, pcd)
            messagebox.showinfo(
                "Save Map", f"Saved {len(pts):,} points to:\n{path}")
        except Exception as e:
            logger.exception("Failed to save map")
            messagebox.showerror("Save Map", f"Error: {e}")

    def _handle_reset_slam(self):
        if self._slam:
            self._slam.reset()

    # ------------------------------------------------------------------
    # Viewer
    # ------------------------------------------------------------------

    def _start_viewer(self):
        if self._viewer and self._viewer.running:
            return
        self._viewer = PointCloudViewer()
        self._viewer.start()
        self._viewer.set_display_mode(
            DisplayMode.SLAM if self._mode == "SLAM" else DisplayMode.VIEW)

    def _update_loop(self):
        if self._panel.closing:
            return

        if self._viewer and self._viewer.running:
            if self._mode == "SLAM" and self._slam and self._receiver:
                state = self._slam.get_state()
                local_map = self._slam.get_local_map()
                alive = self._viewer.update_slam(
                    map_points=local_map,
                    current_frame=state["current_frame"],
                    trajectory=state["trajectory"],
                )
                if not alive:
                    self._handle_disconnect()
                    return
                pose = state["pose"]
                xyz = pose[:3, 3]
                self._panel.update_info_text(
                    f"FPS: {self._receiver.fps:.1f} | "
                    f"Poses: {state['num_poses']} | "
                    f"Pos: ({xyz[0]:+.2f}, {xyz[1]:+.2f}, {xyz[2]:+.2f})")
            elif self._receiver:
                xyz, ref = self._receiver.get_points()
                alive = self._viewer.update(xyz, ref)
                if not alive:
                    self._handle_disconnect()
                    return
                self._panel.update_info(
                    self._receiver.fps, self._receiver.point_count)
            else:
                if not self._viewer.update():
                    self._viewer = None

        if self._conn and not self._conn.connected:
            logger.warning("Connection lost")
            self._handle_disconnect()

        self._panel.schedule(VIEWER_UPDATE_MS, self._update_loop)

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    def run(self):
        logger.info("MID-360 Demo starting")
        self._panel.mainloop()
        self._handle_disconnect()
        logger.info("MID-360 Demo exited")


def main():
    app = App()
    app.run()


if __name__ == "__main__":
    main()
