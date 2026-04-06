"""
MID-360 device connection manager.

Handles:
  - Device discovery via broadcast (port 56000)
  - IP scanning (probe range of IPs)
  - Handshake (register host IP/ports via key-value config)
  - Keep-alive via periodic discovery broadcast
  - Sampling start / stop (work mode control)
"""

import socket
import struct
import threading
import time
import logging
from typing import Callable, List, Optional, Tuple

from . import protocol as proto

logger = logging.getLogger(__name__)

# Discovery broadcast interval (seconds) — also serves as keep-alive
DISCOVERY_INTERVAL = 1.0
# Timeout for waiting command responses
CMD_TIMEOUT = 1.0
# Timeout for IP scan probe
SCAN_TIMEOUT = 0.3


class MID360Connection:
    """Manages the UDP command channel to a single MID-360."""

    def __init__(self, sensor_ip: str, host_ip: str):
        self.sensor_ip = sensor_ip
        self.host_ip = host_ip

        self._seq = 0
        self._cmd_sock: Optional[socket.socket] = None
        self._discovery_thread: Optional[threading.Thread] = None
        self._discovery_stop = threading.Event()
        self._connected = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        """Perform handshake (config write) with the sensor."""
        try:
            self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cmd_sock.settimeout(CMD_TIMEOUT)
            self._cmd_sock.bind((self.host_ip, proto.PORT_HOST_CMD))

            # Send handshake (register host IP/ports)
            if not self._handshake():
                self.disconnect()
                return False

            self._connected = True
            # Start periodic discovery as keep-alive
            self._start_discovery_keepalive()
            logger.info("Connected to MID-360 at %s", self.sensor_ip)
            return True
        except Exception:
            logger.exception("Connection failed")
            self.disconnect()
            return False

    def disconnect(self):
        """Stop keep-alive and close sockets."""
        self._connected = False
        self._discovery_stop.set()
        if self._discovery_thread and self._discovery_thread.is_alive():
            self._discovery_thread.join(timeout=3)
        if self._cmd_sock:
            try:
                self._cmd_sock.close()
            except Exception:
                pass
            self._cmd_sock = None
        logger.info("Disconnected from MID-360")

    def start_sampling(self) -> bool:
        """Set work mode to Normal (start point cloud streaming)."""
        return self._set_work_mode(proto.WorkMode.NORMAL)

    def stop_sampling(self) -> bool:
        """Set work mode to Wake-Up/Standby (stop streaming)."""
        return self._set_work_mode(proto.WorkMode.WAKE_UP)

    # ------------------------------------------------------------------
    # Protocol commands
    # ------------------------------------------------------------------

    def _next_seq(self) -> int:
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return seq

    def _send_cmd(self, cmd_id: int, data: bytes = b'') -> Optional[bytes]:
        """Send a command and wait for ACK. Returns ACK data payload or None."""
        pkt = proto.build_cmd_packet(cmd_id, self._next_seq(), data)
        try:
            self._cmd_sock.sendto(pkt, (self.sensor_ip, proto.PORT_COMMAND))
            resp_raw, addr = self._cmd_sock.recvfrom(4096)
            pkt_resp = proto.parse_cmd_packet(resp_raw)
            if pkt_resp and pkt_resp.cmd_type == proto.CMD_TYPE_ACK:
                return pkt_resp.data
            return None
        except socket.timeout:
            return None
        except Exception:
            logger.exception("Command send failed")
            return None

    def _handshake(self) -> bool:
        """
        Register host IP/ports with the sensor via config write (cmd 0x0100).
        Sends 3 key-value pairs: status push, point cloud, IMU destinations.
        """
        ip_bytes = socket.inet_aton(self.host_ip)

        # Key 0x0005: Status push destination
        kv_push = proto.build_kv_entry(
            proto.CfgKey.STATE_INFO_HOST_IP_CFG,
            proto.build_host_ip_value(ip_bytes, proto.PORT_HOST_PUSH,
                                      proto.PORT_PUSH))

        # Key 0x0006: Point cloud destination
        kv_pcl = proto.build_kv_entry(
            proto.CfgKey.POINT_DATA_HOST_IP_CFG,
            proto.build_host_ip_value(ip_bytes, proto.PORT_HOST_POINTCLOUD,
                                      proto.PORT_POINTCLOUD))

        # Key 0x0007: IMU destination
        kv_imu = proto.build_kv_entry(
            proto.CfgKey.IMU_HOST_IP_CFG,
            proto.build_host_ip_value(ip_bytes, proto.PORT_HOST_IMU,
                                      proto.PORT_IMU))

        payload = proto.build_config_data([kv_push, kv_pcl, kv_imu])
        ack = self._send_cmd(proto.CmdId.LIDAR_CFG_WRITE, payload)

        if ack is None:
            logger.warning("Handshake: no response from %s", self.sensor_ip)
            return False

        # ACK data: ret_code(u8) + error_key(u16)
        if len(ack) >= 3:
            ret_code = ack[0]
            error_key = struct.unpack('<H', ack[1:3])[0]
            if ret_code == 0:
                return True
            logger.warning("Handshake failed: ret=%d, error_key=0x%04X",
                           ret_code, error_key)
            return False

        if len(ack) >= 1 and ack[0] == 0:
            return True
        return False

    def _set_work_mode(self, mode: int) -> bool:
        """Send work mode change via config write."""
        kv = proto.build_kv_entry(
            proto.CfgKey.WORK_MODE,
            struct.pack('<B', mode))
        payload = proto.build_config_data([kv])
        ack = self._send_cmd(proto.CmdId.LIDAR_CFG_WRITE, payload)
        if ack is None:
            return False
        return len(ack) >= 1 and ack[0] == 0

    # ------------------------------------------------------------------
    # Discovery keep-alive thread
    # ------------------------------------------------------------------

    def _start_discovery_keepalive(self):
        self._discovery_stop.clear()
        self._discovery_thread = threading.Thread(
            target=self._discovery_loop, daemon=True, name="discovery-keepalive")
        self._discovery_thread.start()

    def _discovery_loop(self):
        """Periodically send discovery broadcast (serves as keep-alive)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind((self.host_ip, 0))
        except Exception:
            logger.exception("Failed to create discovery socket")
            return

        try:
            while not self._discovery_stop.is_set():
                pkt = proto.build_cmd_packet(
                    proto.CmdId.LIDAR_SEARCH, self._next_seq())
                try:
                    sock.sendto(pkt, ('255.255.255.255', proto.PORT_DISCOVERY))
                except Exception:
                    pass
                self._discovery_stop.wait(DISCOVERY_INTERVAL)
        finally:
            sock.close()


# ---------------------------------------------------------------------------
# NIC enumeration
# ---------------------------------------------------------------------------

def get_local_interfaces() -> List[Tuple[str, str]]:
    """
    Return list of (interface_name, ip_address) for all IPv4 NICs.
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


# ---------------------------------------------------------------------------
# IP Scanner — probe a range of IPs for MID-360 devices
# ---------------------------------------------------------------------------

def scan_mid360(host_ip: str,
                subnet_prefix: str = "192.168.1",
                ip_range: Tuple[int, int] = (100, 200),
                callback: Optional[Callable[[str], None]] = None,
                stop_event: Optional[threading.Event] = None,
                ) -> List[str]:
    """
    Scan for MID-360 devices using broadcast discovery.
    Sends broadcast to 255.255.255.255:56000 and listens for responses.
    The ip_range parameter is ignored (broadcast finds all devices).
    Returns list of IPs that responded.
    """
    found: List[str] = []
    found_lock = threading.Lock()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)
        sock.bind((host_ip, 0))
    except Exception:
        logger.exception("Failed to create scan socket")
        return found

    try:
        # Send multiple broadcast discovery packets
        for attempt in range(5):
            if stop_event and stop_event.is_set():
                break
            pkt = proto.build_cmd_packet(
                proto.CmdId.LIDAR_SEARCH, attempt)
            try:
                sock.sendto(pkt, ('255.255.255.255', proto.PORT_DISCOVERY))
            except Exception:
                pass

            # Listen for responses
            deadline = time.time() + 0.8
            while time.time() < deadline:
                if stop_event and stop_event.is_set():
                    break
                try:
                    resp, addr = sock.recvfrom(4096)
                    pkt_resp = proto.parse_cmd_packet(resp)
                    if (pkt_resp and pkt_resp.sof == proto.FRAME_SOF
                            and pkt_resp.cmd_type == proto.CMD_TYPE_ACK):
                        det = proto.parse_detection_data(pkt_resp.data)
                        if det:
                            ip = det.lidar_ip
                            sn = det.serial_number
                            logger.info("Found: %s (SN=%s, type=%d)",
                                        ip, sn, det.dev_type)
                        else:
                            ip = addr[0]
                        with found_lock:
                            if ip not in found:
                                found.append(ip)
                                if callback:
                                    callback(ip)
                except socket.timeout:
                    break
                except Exception:
                    break
    finally:
        sock.close()

    return found
