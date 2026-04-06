"""
Livox SDK2 communication protocol definitions for MID-360.

Reference: Livox-SDK2 source code (sdk_core/comm/sdk_protocol.h, define.h,
           include/livox_lidar_def.h)
  - All data is little-endian
  - Control commands use a 24-byte header with CRC-16 + CRC-32
  - Point cloud data uses a separate 36-byte header on port 56300
"""

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

# ---------------------------------------------------------------------------
# UDP Ports (MID-360 specific, from define.h lines 191-202)
# ---------------------------------------------------------------------------
PORT_DISCOVERY = 56000       # LiDAR discovery (broadcast)
PORT_COMMAND = 56100         # LiDAR control command port
PORT_HOST_CMD = 56101        # Default host command port
PORT_PUSH = 56200            # LiDAR -> host status push
PORT_HOST_PUSH = 56201       # Default host push port
PORT_POINTCLOUD = 56300      # LiDAR -> host point cloud
PORT_HOST_POINTCLOUD = 56301 # Default host point cloud port
PORT_IMU = 56400             # LiDAR -> host IMU data
PORT_HOST_IMU = 56401        # Default host IMU port
PORT_LOG = 56500             # Log data

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FRAME_SOF = 0xAA             # Start of Frame (NOT 0xAC)
SDK_VERSION = 0x00           # kSdkVer

# ---------------------------------------------------------------------------
# Command IDs (from define.h)
# ---------------------------------------------------------------------------

class CmdId(IntEnum):
    LIDAR_SEARCH        = 0x0000  # Device discovery (broadcast)
    LIDAR_CFG_WRITE     = 0x0100  # Write config params (handshake, work mode, etc.)
    LIDAR_CFG_READ      = 0x0101  # Read config params
    LIDAR_PUSH_MSG      = 0x0102  # Status push from LiDAR
    LIDAR_REBOOT        = 0x0200  # Reboot command


# Cmd type
CMD_TYPE_REQ = 0x00
CMD_TYPE_ACK = 0x01

# Sender type
SENDER_HOST  = 0x00
SENDER_LIDAR = 0x01

# ---------------------------------------------------------------------------
# Device types
# ---------------------------------------------------------------------------

class DeviceType(IntEnum):
    HUB      = 0
    MID40    = 1
    TELE     = 2
    HORIZON  = 3
    MID70    = 6
    AVIA     = 7
    MID360   = 9
    HAP      = 10
    PA       = 11

# ---------------------------------------------------------------------------
# Key IDs for config commands (from livox_lidar_def.h)
# ---------------------------------------------------------------------------

class CfgKey(IntEnum):
    PCL_DATA_TYPE              = 0x0000
    PATTERN_MODE               = 0x0001
    DUAL_EMIT_EN               = 0x0002
    POINT_SEND_EN              = 0x0003
    LIDAR_IP_CFG               = 0x0004
    STATE_INFO_HOST_IP_CFG     = 0x0005
    POINT_DATA_HOST_IP_CFG     = 0x0006
    IMU_HOST_IP_CFG            = 0x0007
    INSTALL_ATTITUDE           = 0x0012
    BLIND_SPOT_SET             = 0x0013
    WORK_MODE                  = 0x001A
    GLASS_HEAT                 = 0x001B
    IMU_DATA_EN                = 0x001C
    FUSA_EN                    = 0x001D
    WORK_MODE_AFTER_BOOT       = 0x0020
    SN                         = 0x8000
    CUR_WORK_STATE             = 0x8006
    STATUS_CODE                = 0x800D


class WorkMode(IntEnum):
    NORMAL         = 0x01  # Sampling active
    WAKE_UP        = 0x02  # Standby / idle
    SLEEP          = 0x03
    ERROR          = 0x04
    POWER_ON_SELF  = 0x05
    MOTOR_STARTING = 0x06
    MOTOR_STOPPING = 0x07
    UPGRADE        = 0x08


# ---------------------------------------------------------------------------
# CRC-16 / CCITT-FALSE  (poly=0x1021, init=0xFFFF, refin=false, refout=false)
# Used to protect the 24-byte command header (bytes 0..17 -> stored at offset 18)
# ---------------------------------------------------------------------------

_CRC16_TABLE = None

def _init_crc16_table():
    global _CRC16_TABLE
    _CRC16_TABLE = [0] * 256
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
        _CRC16_TABLE[i] = crc


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    if _CRC16_TABLE is None:
        _init_crc16_table()
    crc = 0xFFFF
    for b in data:
        crc = ((_CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]) ^ (crc << 8)) & 0xFFFF
    return crc


def crc32(data: bytes) -> int:
    """Standard CRC-32 (ISO-HDLC / Ethernet / zlib)."""
    return zlib.crc32(data) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Control command packet (24-byte header + variable data)
# From sdk_protocol.h — SdkPreamble / SdkPacket, #pragma pack(1)
#
# Offset  Size  Field
# 0       1     sof           (0xAA)
# 1       1     version       (0x00)
# 2       2     length        (total frame = 24 + data_len)
# 4       4     seq_num       (uint32)
# 8       2     cmd_id        (uint16)
# 10      1     cmd_type      (0=REQ, 1=ACK)
# 11      1     sender_type   (0=Host, 1=LiDAR)
# 12      6     rsvd          (zeros)
# 18      2     crc16_h       (CRC-16 of bytes [0..17])
# 20      4     crc32_d       (CRC-32 of data payload only; 0 if no data)
# 24      N     data          (variable payload)
# ---------------------------------------------------------------------------

CMD_HEADER_FORMAT = '<BBHIHBB6sHI'
CMD_HEADER_SIZE = struct.calcsize(CMD_HEADER_FORMAT)  # 24 bytes


@dataclass
class CmdPacket:
    sof: int
    version: int
    length: int
    seq_num: int
    cmd_id: int
    cmd_type: int      # 0=REQ, 1=ACK
    sender_type: int   # 0=Host, 1=LiDAR
    crc16_h: int
    crc32_d: int
    data: bytes


def build_cmd_packet(cmd_id: int, seq_num: int, data: bytes = b'',
                     cmd_type: int = CMD_TYPE_REQ,
                     sender_type: int = SENDER_HOST) -> bytes:
    """Build a complete SDK2 command packet."""
    total_len = CMD_HEADER_SIZE + len(data)

    # Pack header without CRC fields first (to compute CRC-16)
    partial = struct.pack('<BBHIHBB6s',
                          FRAME_SOF, SDK_VERSION, total_len,
                          seq_num, cmd_id, cmd_type, sender_type,
                          b'\x00' * 6)
    # CRC-16 over first 18 bytes
    crc16_val = crc16_ccitt(partial)
    # CRC-32 over data payload (0 if empty)
    crc32_val = crc32(data) if data else 0

    header = partial + struct.pack('<HI', crc16_val, crc32_val)
    return header + data


def parse_cmd_packet(raw: bytes) -> Optional[CmdPacket]:
    """Parse a received command packet."""
    if len(raw) < CMD_HEADER_SIZE:
        return None
    fields = struct.unpack(CMD_HEADER_FORMAT, raw[:CMD_HEADER_SIZE])
    sof, ver, length, seq, cmd_id, cmd_type, sender, rsvd, crc16_h, crc32_d = fields
    if sof != FRAME_SOF:
        return None
    data = raw[CMD_HEADER_SIZE:]
    return CmdPacket(sof=sof, version=ver, length=length, seq_num=seq,
                     cmd_id=cmd_id, cmd_type=cmd_type, sender_type=sender,
                     crc16_h=crc16_h, crc32_d=crc32_d, data=data)


# ---------------------------------------------------------------------------
# Key-Value parameter helpers (for cmd 0x0100 / 0x0101)
# ---------------------------------------------------------------------------

def build_kv_entry(key: int, value: bytes) -> bytes:
    """Build a single key-value entry: key(u16) + length(u16) + value."""
    return struct.pack('<HH', key, len(value)) + value


def build_host_ip_value(host_ip_bytes: bytes, host_port: int,
                        lidar_port: int) -> bytes:
    """Build an 8-byte HostIpInfoValue: ip(4B) + host_port(u16) + lidar_port(u16)."""
    return host_ip_bytes + struct.pack('<HH', host_port, lidar_port)


def build_config_data(kv_entries: list[bytes]) -> bytes:
    """
    Build config command data payload:
      key_num(u16) + rsvd(u16) + concatenated kv entries
    """
    header = struct.pack('<HH', len(kv_entries), 0)
    return header + b''.join(kv_entries)


# ---------------------------------------------------------------------------
# Discovery response (DetectionData from define.h line 132)
# ---------------------------------------------------------------------------
# Data payload of discovery ACK:
# Offset  Size  Field
# 0       1     ret_code    (0=success)
# 1       1     dev_type    (9=Mid-360)
# 2       16    sn          (serial number, ASCII)
# 18      4     lidar_ip    (4 bytes)
# 22      2     cmd_port    (uint16)

DETECTION_DATA_FORMAT = '<BB16s4sH'
DETECTION_DATA_SIZE = struct.calcsize(DETECTION_DATA_FORMAT)  # 24 bytes


@dataclass
class DetectionData:
    ret_code: int
    dev_type: int
    serial_number: str
    lidar_ip: str
    cmd_port: int


def parse_detection_data(data: bytes) -> Optional[DetectionData]:
    if len(data) < DETECTION_DATA_SIZE:
        return None
    ret, dev, sn_raw, ip_raw, port = struct.unpack(
        DETECTION_DATA_FORMAT, data[:DETECTION_DATA_SIZE])
    sn = sn_raw.rstrip(b'\x00').decode('ascii', errors='replace')
    ip = '.'.join(str(b) for b in ip_raw)
    return DetectionData(ret_code=ret, dev_type=dev, serial_number=sn,
                         lidar_ip=ip, cmd_port=port)


# ---------------------------------------------------------------------------
# Point cloud data packet (port 56300)
# From livox_lidar_def.h — LivoxLidarEthernetPacket, 36-byte header
#
# Offset  Size  Field
# 0       1     version
# 1       2     length          (total packet length)
# 3       2     time_interval   (time between points, in 0.1 us)
# 5       2     dot_num         (number of points, typically 96)
# 7       2     udp_cnt         (UDP packet counter)
# 9       1     frame_cnt       (frame counter)
# 10      1     data_type       (1=cart_high, 2=cart_low, 3=spherical)
# 11      1     time_type       (timestamp sync type)
# 12      12    rsvd            (reserved)
# 24      4     crc32           (CRC-32 of bytes 28..end)
# 28      8     timestamp       (first point timestamp, nanoseconds)
# 36      N     point data
# ---------------------------------------------------------------------------

PCL_HEADER_FORMAT = '<BHHHHBBB12sIQ'
PCL_HEADER_SIZE = struct.calcsize(PCL_HEADER_FORMAT)  # 36 bytes


@dataclass
class PointCloudPacket:
    version: int
    length: int
    time_interval: int   # 0.1 us between points
    dot_num: int         # number of points (typically 96)
    udp_cnt: int
    frame_cnt: int
    data_type: int       # 1=cart_high(14B), 2=cart_low(8B), 3=spherical(10B)
    time_type: int
    crc32_val: int
    timestamp_ns: int
    raw_points: bytes


def parse_pointcloud_packet(data: bytes) -> Optional[PointCloudPacket]:
    if len(data) < PCL_HEADER_SIZE:
        return None
    fields = struct.unpack(PCL_HEADER_FORMAT, data[:PCL_HEADER_SIZE])
    (version, length, time_interval, dot_num, udp_cnt,
     frame_cnt, data_type, time_type, rsvd, crc32_val, timestamp_ns) = fields
    return PointCloudPacket(
        version=version, length=length, time_interval=time_interval,
        dot_num=dot_num, udp_cnt=udp_cnt, frame_cnt=frame_cnt,
        data_type=data_type, time_type=time_type,
        crc32_val=crc32_val, timestamp_ns=timestamp_ns,
        raw_points=data[PCL_HEADER_SIZE:]
    )


# ---------------------------------------------------------------------------
# Point data formats
# ---------------------------------------------------------------------------

# data_type=1: Cartesian high resolution — 14 bytes/point
#   x(i32 mm), y(i32 mm), z(i32 mm), reflectivity(u8), tag(u8)
POINT_CART_HIGH_FORMAT = '<iiiBB'
POINT_CART_HIGH_SIZE = 14  # struct.calcsize gives 14

# data_type=2: Cartesian low resolution — 8 bytes/point
#   x(i16 cm), y(i16 cm), z(i16 cm), reflectivity(u8), tag(u8)
POINT_CART_LOW_FORMAT = '<hhhBB'
POINT_CART_LOW_SIZE = 8

# data_type=3: Spherical — 10 bytes/point
#   depth(u32 mm), theta(u16 0.01deg), phi(u16 0.01deg), reflectivity(u8), tag(u8)
POINT_SPHERE_FORMAT = '<IHHBB'
POINT_SPHERE_SIZE = 10


def parse_points_numpy(raw: bytes, data_type: int, dot_num: int):
    """
    Parse raw point bytes into (xyz_Nx3_meters, reflectivity_N).
    Supports data_type 1 (high-res cartesian) and 2 (low-res cartesian).
    """
    import numpy as np

    if data_type == 1:
        point_size = POINT_CART_HIGH_SIZE
        dt = np.dtype([
            ('x', '<i4'), ('y', '<i4'), ('z', '<i4'),
            ('reflectivity', 'u1'), ('tag', 'u1')
        ])
        n = min(dot_num, len(raw) // point_size)
        if n == 0:
            return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.uint8)
        arr = np.frombuffer(raw[:n * point_size], dtype=dt)
        xyz = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        xyz /= 1000.0  # mm -> meters
        return xyz, arr['reflectivity'].copy()

    elif data_type == 2:
        point_size = POINT_CART_LOW_SIZE
        dt = np.dtype([
            ('x', '<i2'), ('y', '<i2'), ('z', '<i2'),
            ('reflectivity', 'u1'), ('tag', 'u1')
        ])
        n = min(dot_num, len(raw) // point_size)
        if n == 0:
            return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.uint8)
        arr = np.frombuffer(raw[:n * point_size], dtype=dt)
        xyz = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        xyz /= 100.0  # cm -> meters
        return xyz, arr['reflectivity'].copy()

    elif data_type == 3:
        point_size = POINT_SPHERE_SIZE
        dt = np.dtype([
            ('depth', '<u4'), ('theta', '<u2'), ('phi', '<u2'),
            ('reflectivity', 'u1'), ('tag', 'u1')
        ])
        n = min(dot_num, len(raw) // point_size)
        if n == 0:
            return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.uint8)
        arr = np.frombuffer(raw[:n * point_size], dtype=dt)
        depth = arr['depth'].astype(np.float64) / 1000.0  # mm -> m
        theta = np.radians(arr['theta'].astype(np.float64) * 0.01)
        phi = np.radians(arr['phi'].astype(np.float64) * 0.01)
        x = depth * np.sin(theta) * np.cos(phi)
        y = depth * np.sin(theta) * np.sin(phi)
        z = depth * np.cos(theta)
        xyz = np.column_stack([x, y, z])
        return xyz, arr['reflectivity'].copy()

    else:
        return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.uint8)
