"""
MID-360 device connection manager.

Handles:
  - IP scanning (ping-like probe via UDP on command port)
  - Handshake
  - Heartbeat keep-alive
  - Sampling start / stop
"""

import socket
import struct
import threading
import time
import logging
from typing import Callable, List, Optional, Tuple

from . import protocol as proto

logger = logging.getLogger(__name__)

# Heartbeat interval (seconds) — must be < 3 s or device disconnects
HEARTBEAT_INTERVAL = 1.0
# Timeout for waiting command responses
CMD_TIMEOUT = 1.0
# Timeout for IP scan probe
SCAN_TIMEOUT = 0.3


class MID360Connection:
    """Manages the UDP command channel to a single MID-360."""

    def __init__(self, sensor_ip: str, host_ip: str,
                 cmd_port: int = proto.PORT_COMMAND):
        self.sensor_ip = sensor_ip
        self.host_ip = host_ip
        self.cmd_port = cmd_port

        self._seq = 0
        self._sock: Optional[socket.socket] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._heartbeat_stop = threading.Event()
        self._connected = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        """Perform handshake with the sensor."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(CMD_TIMEOUT)
            # Bind to any available port on host
            self._sock.bind((self.host_ip, 0))

            # Send handshake
            if not self._handshake():
                self.disconnect()
                return False

            self._connected = True
            self._start_heartbeat()
            logger.info("Connected to MID-360 at %s", self.sensor_ip)
            return True
        except Exception:
            logger.exception("Connection failed")
            self.disconnect()
            return False

    def disconnect(self):
        """Stop heartbeat and close socket."""
        self._connected = False
        self._heartbeat_stop.set()
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=3)
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        logger.info("Disconnected from MID-360")

    def start_sampling(self) -> bool:
        """Send sampling-start command (sample_ctrl = 1)."""
        return self._sampling_ctrl(1)

    def stop_sampling(self) -> bool:
        """Send sampling-stop command (sample_ctrl = 0)."""
        return self._sampling_ctrl(0)

    # ------------------------------------------------------------------
    # Protocol commands
    # ------------------------------------------------------------------

    def _next_seq(self) -> int:
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            return seq

    def _send_cmd(self, cmd_set: int, cmd_id: int,
                  payload: bytes = b'') -> Optional[bytes]:
        """Send a command and wait for ACK. Returns ACK payload or None."""
        pkt = proto.build_packet(0, self._next_seq(), cmd_set, cmd_id, payload)
        try:
            self._sock.sendto(pkt, (self.sensor_ip, self.cmd_port))
            resp, addr = self._sock.recvfrom(2048)
            hdr = proto.parse_header(resp)
            if hdr and hdr.cmd_type == 1:  # ACK
                return resp[proto.HEADER_SIZE:-proto.CRC16_SIZE]
            return None
        except socket.timeout:
            return None
        except Exception:
            logger.exception("Command send failed")
            return None

    def _handshake(self) -> bool:
        """
        Handshake: tell the sensor which host IP/ports to send data to.
        Payload = host_ip(4B) + pointcloud_port(2B) + cmd_port(2B) + imu_port(2B)
        """
        ip_bytes = socket.inet_aton(self.host_ip)
        local_port = self._sock.getsockname()[1]
        payload = ip_bytes + struct.pack('<HHH',
                                         proto.PORT_POINTCLOUD,
                                         local_port,
                                         proto.PORT_IMU)
        ack = self._send_cmd(proto.CmdSet.GENERAL,
                             proto.GeneralCmdId.HANDSHAKE,
                             payload)
        if ack is None:
            logger.warning("Handshake: no response from %s", self.sensor_ip)
            return False
        # ACK payload: ret_code (uint8), 0=success
        if len(ack) >= 1 and ack[0] == 0:
            return True
        logger.warning("Handshake failed, ret_code=%d", ack[0] if ack else -1)
        return False

    def _heartbeat(self) -> bool:
        ack = self._send_cmd(proto.CmdSet.GENERAL,
                             proto.GeneralCmdId.HEARTBEAT)
        return ack is not None

    def _sampling_ctrl(self, start: int) -> bool:
        payload = struct.pack('<B', start)
        ack = self._send_cmd(proto.CmdSet.GENERAL,
                             proto.GeneralCmdId.SAMPLING_CTRL,
                             payload)
        if ack is None:
            return False
        return len(ack) >= 1 and ack[0] == 0

    # ------------------------------------------------------------------
    # Heartbeat thread
    # ------------------------------------------------------------------

    def _start_heartbeat(self):
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="heartbeat")
        self._heartbeat_thread.start()

    def _heartbeat_loop(self):
        fail_count = 0
        while not self._heartbeat_stop.is_set():
            if not self._heartbeat():
                fail_count += 1
                if fail_count >= 3:
                    logger.error("Heartbeat lost — disconnecting")
                    self._connected = False
                    break
            else:
                fail_count = 0
            self._heartbeat_stop.wait(HEARTBEAT_INTERVAL)


# ---------------------------------------------------------------------------
# IP Scanner — probe a range of IPs for MID-360 devices
# ---------------------------------------------------------------------------

def get_local_interfaces() -> List[Tuple[str, str]]:
    """
    Return list of (interface_name_or_ip, ip_address) for all IPv4 NICs.
    Works on Windows and Linux.
    """
    results = []
    try:
        import psutil
        for name, addrs in psutil.net_if_addrs().items():
            for addr in addrs:
                if addr.family == socket.AF_INET:
                    results.append((name, addr.address))
    except ImportError:
        # Fallback: use socket to guess local IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            results.append(("default", ip))
        except Exception:
            results.append(("localhost", "127.0.0.1"))
    return results


def scan_mid360(host_ip: str,
                subnet_prefix: str = "192.168.1",
                ip_range: Tuple[int, int] = (100, 200),
                callback: Optional[Callable[[str], None]] = None,
                stop_event: Optional[threading.Event] = None,
                ) -> List[str]:
    """
    Scan a range of IPs by sending a handshake probe.
    Returns list of IPs that responded.

    Uses parallel threads for speed.
    """
    found: List[str] = []
    found_lock = threading.Lock()

    def probe(ip: str):
        if stop_event and stop_event.is_set():
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(SCAN_TIMEOUT)
            sock.bind((host_ip, 0))
            # Send a heartbeat as a lightweight probe
            pkt = proto.build_packet(
                0, 0, proto.CmdSet.GENERAL,
                proto.GeneralCmdId.HEARTBEAT)
            sock.sendto(pkt, (ip, proto.PORT_COMMAND))
            try:
                resp, _ = sock.recvfrom(1024)
                hdr = proto.parse_header(resp)
                if hdr and hdr.sof == proto.FRAME_SOF:
                    with found_lock:
                        found.append(ip)
                    if callback:
                        callback(ip)
            except socket.timeout:
                pass
            finally:
                sock.close()
        except Exception:
            pass

    threads = []
    start, end = ip_range
    for i in range(start, end + 1):
        if stop_event and stop_event.is_set():
            break
        ip = f"{subnet_prefix}.{i}"
        t = threading.Thread(target=probe, args=(ip,), daemon=True)
        threads.append(t)
        t.start()
        # Limit concurrency
        if len(threads) >= 20:
            for t in threads:
                t.join(timeout=SCAN_TIMEOUT + 0.2)
            threads.clear()

    for t in threads:
        t.join(timeout=SCAN_TIMEOUT + 0.2)

    return found
