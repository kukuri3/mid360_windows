"""
Livox SDK2 communication protocol definitions for MID-360.

Reference: Livox SDK2 Communication Protocol
  - All data is little-endian
  - UDP ports: discovery=56000, command=56100, push=56200,
               pointcloud=56300, imu=56400, log=56500
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

# ---------------------------------------------------------------------------
# UDP Ports
# ---------------------------------------------------------------------------
PORT_DISCOVERY = 56000
PORT_COMMAND = 56100
PORT_PUSH = 56200
PORT_POINTCLOUD = 56300
PORT_IMU = 56400
PORT_LOG = 56500

# ---------------------------------------------------------------------------
# Magic / SOF
# ---------------------------------------------------------------------------
FRAME_SOF = 0xAC  # Start of Frame byte

# ---------------------------------------------------------------------------
# Command Sets & IDs
# ---------------------------------------------------------------------------

class CmdSet(IntEnum):
    GENERAL = 0x00
    LIDAR   = 0x01
    HUB     = 0x02


class GeneralCmdId(IntEnum):
    DEVICE_TYPE_QUERY = 0x00  # Broadcast / discovery
    HANDSHAKE         = 0x01
    DEVICE_INFO       = 0x02
    HEARTBEAT         = 0x03
    SAMPLING_CTRL     = 0x04
    REBOOT            = 0x06
    WRITE_PARAMS      = 0x08
    READ_PARAMS       = 0x09
    PUSH_MSG          = 0x0A
    LOG_CTRL          = 0x0C


class DeviceType(IntEnum):
    HUB          = 0
    MID40        = 1
    TELE         = 2
    HORIZON      = 3
    MID70        = 6
    AVIA         = 7
    MID360       = 9
    HAP          = 10
    PA           = 11


# ---------------------------------------------------------------------------
# Packet header  (24 bytes for SDK2 / HAP protocol)
# ---------------------------------------------------------------------------
# Offset  Size  Field
# 0       1     sof           (0xAC)
# 1       1     version       (protocol version, typically 0x01)
# 2       2     length        (total packet length including header)
# 4       1     cmd_type      (0=request, 1=ack/response)
# 5       2     seq_num       (sequence number)
# 7       1     cmd_set       (command set)
# 8       1     cmd_id        (command id within set)
# 9       ...   payload       (variable)
# last 2  2     crc16         (CRC-16 over entire packet excl crc16)
#
# Note: The actual SDK2 header is slightly more complex with a preamble
# and frame header. For simplicity we use the following working format
# based on observed packets.

HEADER_FORMAT = '<BBHBHBB'  # sof, ver, length, cmd_type, seq, cmd_set, cmd_id
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 9 bytes
CRC16_SIZE = 2


@dataclass
class PacketHeader:
    sof: int
    version: int
    length: int
    cmd_type: int    # 0=req, 1=ack
    seq_num: int
    cmd_set: int
    cmd_id: int


def pack_header(cmd_type: int, seq_num: int, cmd_set: int, cmd_id: int,
                payload_len: int) -> bytes:
    total_len = HEADER_SIZE + payload_len + CRC16_SIZE
    return struct.pack(HEADER_FORMAT,
                       FRAME_SOF, 0x01, total_len, cmd_type,
                       seq_num, cmd_set, cmd_id)


def parse_header(data: bytes) -> Optional[PacketHeader]:
    if len(data) < HEADER_SIZE:
        return None
    fields = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    return PacketHeader(*fields)


# ---------------------------------------------------------------------------
# CRC-16 (CCITT, used by Livox)
# ---------------------------------------------------------------------------
_CRC16_TABLE = None

def _init_crc16_table():
    global _CRC16_TABLE
    _CRC16_TABLE = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
        _CRC16_TABLE.append(crc & 0xFFFF)

def crc16(data: bytes) -> int:
    if _CRC16_TABLE is None:
        _init_crc16_table()
    crc = 0xFFFF
    for b in data:
        crc = (_CRC16_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)) & 0xFFFF
    return crc


def build_packet(cmd_type: int, seq_num: int, cmd_set: int, cmd_id: int,
                 payload: bytes = b'') -> bytes:
    header = pack_header(cmd_type, seq_num, cmd_set, cmd_id, len(payload))
    body = header + payload
    checksum = crc16(body)
    return body + struct.pack('<H', checksum)


def verify_packet(data: bytes) -> bool:
    if len(data) < HEADER_SIZE + CRC16_SIZE:
        return False
    body = data[:-CRC16_SIZE]
    expected = struct.unpack('<H', data[-CRC16_SIZE:])[0]
    return crc16(body) == expected


# ---------------------------------------------------------------------------
# Point cloud data frame (port 56300)
# ---------------------------------------------------------------------------
# The point cloud data uses a different, simpler frame format:
#
# Offset  Size  Field
# 0       1     version
# 1       2     slot_id + lidar_id
# 3       1     reserved
# 4       4     status_code
# 8       1     timestamp_type
# 9       1     data_type
# 10      8     timestamp (ns)
# 18      N     point data
#
# data_type determines point format:
#   1 = Cartesian (x,y,z int32 mm + reflectivity uint8 + tag uint8) = 14 bytes/pt
#   2 = Spherical
#   3 = Cartesian high (x,y,z int32 + reflectivity uint16 + tag uint8) = 15 bytes/pt

PCLOUD_HEADER_FORMAT = '<BHBIBB8s'
PCLOUD_HEADER_SIZE = struct.calcsize(PCLOUD_HEADER_FORMAT)  # 18 bytes

# Cartesian point: x(i32) y(i32) z(i32) reflectivity(u8) tag(u8)
POINT_CARTESIAN_FORMAT = '<iiiBB'
POINT_CARTESIAN_SIZE = struct.calcsize(POINT_CARTESIAN_FORMAT)  # 14 bytes


@dataclass
class PointCloudFrame:
    version: int
    slot_id: int
    status_code: int
    timestamp_type: int
    data_type: int
    timestamp_ns: int
    raw_points: bytes  # raw point data


def parse_pointcloud_header(data: bytes) -> Optional[PointCloudFrame]:
    if len(data) < PCLOUD_HEADER_SIZE:
        return None
    fields = struct.unpack(PCLOUD_HEADER_FORMAT, data[:PCLOUD_HEADER_SIZE])
    version, slot_lidar, reserved, status, ts_type, data_type, ts_bytes = fields
    timestamp_ns = struct.unpack('<Q', ts_bytes)[0]
    return PointCloudFrame(
        version=version,
        slot_id=slot_lidar,
        status_code=status,
        timestamp_type=ts_type,
        data_type=data_type,
        timestamp_ns=timestamp_ns,
        raw_points=data[PCLOUD_HEADER_SIZE:]
    )


def parse_cartesian_points(raw: bytes):
    """Parse raw bytes into list of (x_mm, y_mm, z_mm, reflectivity, tag)."""
    n = len(raw) // POINT_CARTESIAN_SIZE
    points = []
    for i in range(n):
        offset = i * POINT_CARTESIAN_SIZE
        x, y, z, ref, tag = struct.unpack(
            POINT_CARTESIAN_FORMAT,
            raw[offset:offset + POINT_CARTESIAN_SIZE]
        )
        points.append((x, y, z, ref, tag))
    return points


def parse_cartesian_points_numpy(raw: bytes):
    """Parse raw bytes into numpy array (Nx3 float, meters) + reflectivity."""
    import numpy as np
    n = len(raw) // POINT_CARTESIAN_SIZE
    if n == 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.uint8)

    # Use numpy for fast parsing
    dt = np.dtype([
        ('x', '<i4'), ('y', '<i4'), ('z', '<i4'),
        ('reflectivity', 'u1'), ('tag', 'u1')
    ])
    arr = np.frombuffer(raw[:n * POINT_CARTESIAN_SIZE], dtype=dt)
    xyz = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64) / 1000.0
    return xyz, arr['reflectivity']
