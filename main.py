"""
MID-360 Point Cloud Demo — main entry point.

Launches the tkinter control panel and Open3D viewer,
wiring them together with the MID-360 connection and receiver.
"""

import logging

from mid360.connection import MID360Connection
from mid360.pointcloud_receiver import PointCloudReceiver
from mid360.viewer import PointCloudViewer, ColorMode
from gui.control_panel import ControlPanel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

VIEWER_UPDATE_MS = 33  # ~30 fps

# Map GUI label -> ColorMode enum
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

        self._panel = ControlPanel(
            on_connect=self._handle_connect,
            on_disconnect=self._handle_disconnect,
            on_accum_changed=self._handle_accum_changed,
            on_point_size_changed=self._handle_point_size_changed,
            on_color_mode_changed=self._handle_color_mode_changed,
        )

        self._panel.schedule(VIEWER_UPDATE_MS, self._update_loop)

    # ------------------------------------------------------------------
    # Connection callbacks (may run from worker threads)
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
            receiver.accum_seconds = 0.5  # match GUI default
            receiver.start()

            if not conn.start_sampling():
                logger.warning("start_sampling command failed "
                               "(may already be streaming)")

            self._conn = conn
            self._receiver = receiver

            self._panel.root.after(0, self._start_viewer)
            self._panel.root.after(0, lambda: self._panel.set_connected(True))

        except Exception:
            logger.exception("Connection error")
            self._panel.root.after(0, lambda: self._panel.set_connected(False))

    def _handle_disconnect(self):
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
    # Display setting callbacks
    # ------------------------------------------------------------------

    def _handle_accum_changed(self, seconds: float):
        if self._receiver:
            self._receiver.accum_seconds = seconds

    def _handle_point_size_changed(self, size: float):
        if self._viewer:
            self._viewer.point_size = size

    def _handle_color_mode_changed(self, mode_label: str):
        if self._viewer:
            mode = _COLOR_MODE_MAP.get(mode_label, ColorMode.HEIGHT_Z)
            self._viewer.color_mode = mode

    # ------------------------------------------------------------------
    # Viewer
    # ------------------------------------------------------------------

    def _start_viewer(self):
        if self._viewer and self._viewer.running:
            return
        self._viewer = PointCloudViewer()
        self._viewer.start()

    def _update_loop(self):
        if self._panel.closing:
            return

        if self._viewer and self._viewer.running and self._receiver:
            xyz, ref = self._receiver.get_points()
            alive = self._viewer.update(xyz, ref)
            if not alive:
                self._handle_disconnect()
                return

            self._panel.update_info(
                self._receiver.fps, self._receiver.point_count)

        elif self._viewer and self._viewer.running:
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
