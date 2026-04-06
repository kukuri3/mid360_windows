"""
MID-360 Point Cloud Demo — main entry point.

Launches the tkinter control panel and Open3D viewer,
wiring them together with the MID-360 connection and receiver.
"""

import logging
import sys

from mid360.connection import MID360Connection
from mid360.pointcloud_receiver import PointCloudReceiver
from mid360.viewer import PointCloudViewer
from gui.control_panel import ControlPanel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# Display refresh interval (ms)
VIEWER_UPDATE_MS = 33  # ~30 fps


class App:
    def __init__(self):
        self._conn = None
        self._receiver = None
        self._viewer = None

        self._panel = ControlPanel(
            on_connect=self._handle_connect,
            on_disconnect=self._handle_disconnect,
        )

        # Start the periodic viewer update loop
        self._panel.schedule(VIEWER_UPDATE_MS, self._update_loop)

    # ------------------------------------------------------------------
    # Connection callbacks (may run from worker threads)
    # ------------------------------------------------------------------

    def _handle_connect(self, sensor_ip: str, host_ip: str):
        """Called from a worker thread when user clicks Connect."""
        try:
            conn = MID360Connection(sensor_ip, host_ip)
            if not conn.connect():
                self._panel.root.after(
                    0, lambda: self._panel.set_connected(False))
                logger.error("Failed to connect to %s", sensor_ip)
                return

            # Start point cloud receiver
            receiver = PointCloudReceiver(host_ip)
            receiver.start()

            # Start sampling
            if not conn.start_sampling():
                logger.warning("start_sampling command failed "
                               "(may already be streaming)")

            self._conn = conn
            self._receiver = receiver

            # Start viewer on main thread
            self._panel.root.after(0, self._start_viewer)
            self._panel.root.after(0, lambda: self._panel.set_connected(True))

        except Exception:
            logger.exception("Connection error")
            self._panel.root.after(0, lambda: self._panel.set_connected(False))

    def _handle_disconnect(self):
        """Called when user clicks Disconnect or closes the window."""
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
    # Viewer
    # ------------------------------------------------------------------

    def _start_viewer(self):
        if self._viewer and self._viewer.running:
            return
        self._viewer = PointCloudViewer()
        self._viewer.start()

    def _update_loop(self):
        """Periodic callback — update Open3D viewer and stats."""
        if self._panel.closing:
            return

        if self._viewer and self._viewer.running and self._receiver:
            xyz, ref = self._receiver.get_points()
            alive = self._viewer.update(xyz, ref)
            if not alive:
                self._handle_disconnect()
                return

            # Update stats in GUI
            self._panel.update_info(
                self._receiver.fps, self._receiver.point_count)

        elif self._viewer and self._viewer.running:
            # Keep Open3D alive even with no data
            if not self._viewer.update():
                self._viewer = None

        # Check if connection was lost
        if self._conn and not self._conn.connected:
            logger.warning("Connection lost")
            self._handle_disconnect()

        # Reschedule
        self._panel.schedule(VIEWER_UPDATE_MS, self._update_loop)

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    def run(self):
        logger.info("MID-360 Demo starting")
        self._panel.mainloop()
        # Cleanup on exit
        self._handle_disconnect()
        logger.info("MID-360 Demo exited")


def main():
    app = App()
    app.run()


if __name__ == "__main__":
    main()
