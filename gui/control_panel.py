"""
tkinter control panel for MID-360 demo.

Features:
  - NIC (network interface) selector
  - Sensor IP input + scan button
  - Connect / Disconnect
  - Status display (connection state, FPS, point count)
"""

import socket
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import logging
from typing import Callable, List, Optional, Tuple

from mid360.connection import get_local_interfaces, scan_mid360

logger = logging.getLogger(__name__)

# Default subnet for MID-360
DEFAULT_SUBNET = "192.168.1"
DEFAULT_SCAN_START = 100
DEFAULT_SCAN_END = 200


class ControlPanel:
    """tkinter-based control panel for MID-360 connection management."""

    def __init__(self,
                 on_connect: Optional[Callable[[str, str], None]] = None,
                 on_disconnect: Optional[Callable[[], None]] = None):
        """
        Args:
            on_connect: callback(sensor_ip, host_ip) when user clicks Connect
            on_disconnect: callback() when user clicks Disconnect
        """
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect

        self.root = tk.Tk()
        self.root.title("MID-360 Controller")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._scanning = False
        self._scan_stop = threading.Event()
        self._connected = False
        self._closing = False

        self._build_ui()
        self._refresh_interfaces()

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        pad = dict(padx=6, pady=3)
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(sticky="nsew")

        row = 0

        # --- NIC Selection ---
        ttk.Label(frame, text="Network Interface:").grid(
            row=row, column=0, sticky="w", **pad)
        self._nic_var = tk.StringVar()
        self._nic_combo = ttk.Combobox(
            frame, textvariable=self._nic_var, width=35, state="readonly")
        self._nic_combo.grid(row=row, column=1, columnspan=2, sticky="ew", **pad)
        self._nic_combo.bind("<<ComboboxSelected>>", self._on_nic_selected)

        btn_refresh = ttk.Button(frame, text="Refresh",
                                 command=self._refresh_interfaces, width=8)
        btn_refresh.grid(row=row, column=3, **pad)
        row += 1

        # --- Subnet & Scan ---
        ttk.Label(frame, text="Scan Subnet:").grid(
            row=row, column=0, sticky="w", **pad)

        scan_frame = ttk.Frame(frame)
        scan_frame.grid(row=row, column=1, columnspan=2, sticky="ew", **pad)

        self._subnet_var = tk.StringVar(value=DEFAULT_SUBNET)
        ttk.Entry(scan_frame, textvariable=self._subnet_var,
                  width=14).pack(side="left")
        ttk.Label(scan_frame, text=".").pack(side="left")
        self._scan_start_var = tk.StringVar(value=str(DEFAULT_SCAN_START))
        ttk.Entry(scan_frame, textvariable=self._scan_start_var,
                  width=5).pack(side="left")
        ttk.Label(scan_frame, text="-").pack(side="left")
        self._scan_end_var = tk.StringVar(value=str(DEFAULT_SCAN_END))
        ttk.Entry(scan_frame, textvariable=self._scan_end_var,
                  width=5).pack(side="left")

        self._btn_scan = ttk.Button(frame, text="Scan", command=self._on_scan,
                                    width=8)
        self._btn_scan.grid(row=row, column=3, **pad)
        row += 1

        # --- Scan results / IP selection ---
        ttk.Label(frame, text="Sensor IP:").grid(
            row=row, column=0, sticky="w", **pad)
        self._ip_var = tk.StringVar()
        self._ip_combo = ttk.Combobox(
            frame, textvariable=self._ip_var, width=35)
        self._ip_combo.grid(row=row, column=1, columnspan=2, sticky="ew", **pad)
        row += 1

        # --- Connect / Disconnect ---
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=row, column=0, columnspan=4, pady=8)

        self._btn_connect = ttk.Button(
            btn_frame, text="Connect", command=self._on_connect, width=14)
        self._btn_connect.pack(side="left", padx=8)

        self._btn_disconnect = ttk.Button(
            btn_frame, text="Disconnect", command=self._on_disconnect,
            width=14, state="disabled")
        self._btn_disconnect.pack(side="left", padx=8)
        row += 1

        # --- Status ---
        sep = ttk.Separator(frame, orient="horizontal")
        sep.grid(row=row, column=0, columnspan=4, sticky="ew", pady=6)
        row += 1

        self._status_var = tk.StringVar(value="Disconnected")
        ttk.Label(frame, text="Status:").grid(
            row=row, column=0, sticky="w", **pad)
        self._lbl_status = ttk.Label(frame, textvariable=self._status_var,
                                     foreground="gray")
        self._lbl_status.grid(row=row, column=1, columnspan=3, sticky="w", **pad)
        row += 1

        self._info_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self._info_var).grid(
            row=row, column=0, columnspan=4, sticky="w", **pad)

    # ------------------------------------------------------------------
    # NIC management
    # ------------------------------------------------------------------

    def _refresh_interfaces(self):
        self._interfaces: List[Tuple[str, str]] = get_local_interfaces()
        display = [f"{name}  ({ip})" for name, ip in self._interfaces]
        self._nic_combo['values'] = display
        if display:
            self._nic_combo.current(0)
            self._on_nic_selected(None)

    def _on_nic_selected(self, event):
        idx = self._nic_combo.current()
        if idx < 0 or idx >= len(self._interfaces):
            return
        _, ip = self._interfaces[idx]
        # Auto-set subnet from selected NIC
        parts = ip.split('.')
        if len(parts) == 4:
            self._subnet_var.set('.'.join(parts[:3]))

    def _get_host_ip(self) -> Optional[str]:
        idx = self._nic_combo.current()
        if idx < 0 or idx >= len(self._interfaces):
            messagebox.showerror("Error", "No network interface selected")
            return None
        return self._interfaces[idx][1]

    # ------------------------------------------------------------------
    # Scan
    # ------------------------------------------------------------------

    def _on_scan(self):
        if self._scanning:
            # Cancel ongoing scan
            self._scan_stop.set()
            return

        host_ip = self._get_host_ip()
        if not host_ip:
            return

        subnet = self._subnet_var.get().strip()
        try:
            start = int(self._scan_start_var.get())
            end = int(self._scan_end_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid scan range")
            return

        self._scanning = True
        self._scan_stop.clear()
        self._btn_scan.configure(text="Stop")
        self._ip_combo['values'] = []
        self._status_var.set(f"Scanning {subnet}.{start}-{end} ...")
        self._lbl_status.configure(foreground="orange")

        def on_found(ip: str):
            self.root.after(0, self._scan_found, ip)

        def do_scan():
            found = scan_mid360(
                host_ip=host_ip, subnet_prefix=subnet,
                ip_range=(start, end),
                callback=on_found, stop_event=self._scan_stop)
            self.root.after(0, self._scan_done, found)

        threading.Thread(target=do_scan, daemon=True).start()

    def _scan_found(self, ip: str):
        current = list(self._ip_combo['values'])
        if ip not in current:
            current.append(ip)
            self._ip_combo['values'] = current
            self._ip_var.set(ip)
            self._status_var.set(f"Found: {ip}")

    def _scan_done(self, found: List[str]):
        self._scanning = False
        self._btn_scan.configure(text="Scan")
        if found:
            self._status_var.set(f"Scan complete — {len(found)} device(s) found")
            self._lbl_status.configure(foreground="green")
        else:
            self._status_var.set("Scan complete — no devices found")
            self._lbl_status.configure(foreground="red")

    # ------------------------------------------------------------------
    # Connect / Disconnect
    # ------------------------------------------------------------------

    def _on_connect(self):
        ip = self._ip_var.get().strip()
        if not ip:
            messagebox.showerror("Error", "Enter or scan for a sensor IP")
            return
        host_ip = self._get_host_ip()
        if not host_ip:
            return

        self._status_var.set(f"Connecting to {ip} ...")
        self._lbl_status.configure(foreground="orange")
        self._btn_connect.configure(state="disabled")
        self.root.update_idletasks()

        if self.on_connect:
            # Run connection in a thread to avoid blocking GUI
            def do_connect():
                self.on_connect(ip, host_ip)
            threading.Thread(target=do_connect, daemon=True).start()

    def _on_disconnect(self):
        if self.on_disconnect:
            self.on_disconnect()

    def set_connected(self, connected: bool):
        """Called from outside to update UI state."""
        self._connected = connected
        if connected:
            self._btn_connect.configure(state="disabled")
            self._btn_disconnect.configure(state="normal")
            self._status_var.set("Connected")
            self._lbl_status.configure(foreground="green")
        else:
            self._btn_connect.configure(state="normal")
            self._btn_disconnect.configure(state="disabled")
            self._status_var.set("Disconnected")
            self._lbl_status.configure(foreground="gray")
            self._info_var.set("")

    def update_info(self, fps: float, point_count: int):
        """Update the stats display."""
        self._info_var.set(f"FPS: {fps:.1f}  |  Points: {point_count:,}")

    # ------------------------------------------------------------------
    # Main loop integration
    # ------------------------------------------------------------------

    def schedule(self, ms: int, func: Callable):
        """Schedule a repeating callback via tkinter.after()."""
        self.root.after(ms, func)

    def mainloop(self):
        self.root.mainloop()

    def _on_close(self):
        self._closing = True
        if self.on_disconnect and self._connected:
            self.on_disconnect()
        self.root.destroy()

    @property
    def closing(self) -> bool:
        return self._closing
