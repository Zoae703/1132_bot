"""
Binary communication protocol v2 — Python mirror of protocol/shared/protocol.h.

Frame format (little-endian):
  [0xAA 0x55] [version:1] [type:1] [seq:2] [payload_len:2] [payload:N] [crc16:2]

Total overhead: 8 bytes.  Max payload: 240 bytes.
CRC16: CCITT (polynomial 0x1021, initial value 0xFFFF).

This file is the SINGLE SOURCE OF TRUTH for the Python side.
The C header at protocol/shared/protocol.h MUST stay in sync.
"""

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Tuple, List

# ---------------------------------------------------------------------------
#  Frame constants
# ---------------------------------------------------------------------------

MAGIC_0: int = 0xAA
MAGIC_1: int = 0x55
VERSION: int = 0x02
FRAME_OVERHEAD: int = 8
CRC_SIZE: int = 2
MAX_PAYLOAD: int = 240
BUF_SIZE: int = MAX_PAYLOAD + FRAME_OVERHEAD + CRC_SIZE
PROTO_BUF_SIZE: int = BUF_SIZE

# ---------------------------------------------------------------------------
#  Enums (must match protocol.h exactly)
# ---------------------------------------------------------------------------


class MsgType(IntEnum):
    """Message types."""
    NOP = 0x00
    ACK = 0x01
    NACK = 0x02
    ARM = 0x10
    DISARM = 0x11
    EMERGENCY_STOP = 0x12
    RESET_ESTOP = 0x13
    ENTER_MANUAL = 0x14
    EXIT_MANUAL = 0x15
    SET_PWM = 0x20
    SET_ALL_NEUTRAL = 0x21
    FLOAT_ON = 0x30
    FLOAT_OFF = 0x31
    ANGLE_ON = 0x32
    ANGLE_OFF = 0x33
    SET_DEPTH = 0x34
    SET_YAW = 0x35
    SET_MOTION = 0x36
    REQUEST_STATUS = 0x40
    REQUEST_SENSORS = 0x41
    STATUS_REPORT = 0x80
    SENSOR_REPORT = 0x81
    LOG_MESSAGE = 0x90
    SAFETY_EVENT = 0x91
    HEARTBEAT = 0xF0
    HEARTBEAT_ACK = 0xF1


class SafetyState(IntEnum):
    """Robot safety states."""
    DISARMED = 0
    ARMED_IDLE = 1
    ARMED_ACTIVE = 2
    MANUAL_TEST = 3
    COMM_LOST = 4
    EMERGENCY_STOP = 5
    FAULT = 6


class NackReason(IntEnum):
    """NACK reason codes."""
    UNKNOWN = 0
    BAD_CRC = 1
    BAD_STATE = 2
    BAD_CHANNEL = 3
    BAD_PWM_VALUE = 4
    CHANNEL_BUSY = 5
    ESTOP_LOCKED = 6
    NOT_ARMED = 7
    TIMEOUT = 8
    INTERNAL_ERROR = 9
    INVALID_PAYLOAD_LENGTH = 10
    UNSUPPORTED_MESSAGE = 11
    INVALID_VALUE = 12


class NeutralReason(IntEnum):
    """Reason why the motor outputs most recently returned to neutral."""
    NONE = 0
    COMMAND = 1
    PWM_COMMAND_TIMEOUT = 2
    COMM_LOST = 3
    DISARM = 4
    EMERGENCY_STOP = 5
    LAST_CLIENT_DISCONNECTED = 6
    FAULT = 7


class SafetyEventType(IntEnum):
    """Safety event types."""
    COMM_LOST = 0
    COMM_RESTORED = 1
    ESTOP_TRIGGERED = 2
    FAULT = 3
    STATE_CHANGE = 4


# ---------------------------------------------------------------------------
#  CRC16 CCITT
# ---------------------------------------------------------------------------

_CRC16_TABLE: Optional[List[int]] = None


def _make_crc16_table() -> List[int]:
    """Build CRC16-CCITT lookup table."""
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


def crc16(data: bytes) -> int:
    """Compute CRC16-CCITT over bytes."""
    global _CRC16_TABLE
    if _CRC16_TABLE is None:
        _CRC16_TABLE = _make_crc16_table()

    crc = 0xFFFF
    for byte in data:
        idx = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ _CRC16_TABLE[idx]) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
#  Payload dataclasses
# ---------------------------------------------------------------------------


@dataclass
class SetPwm:
    """SET_PWM payload."""
    channel: int = 0
    pwm_us: int = 1500
    timeout_ms: int = 500

    _fmt: str = field(default="<BBhH", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.channel & 0xFF, 0,
                           self.pwm_us, self.timeout_ms & 0xFFFF)

    @classmethod
    def unpack(cls, data: bytes) -> "SetPwm":
        ch, _, pwm, timeout = struct.unpack(cls._fmt, data)
        return cls(channel=ch, pwm_us=pwm, timeout_ms=timeout)


@dataclass
class SetDepth:
    """SET_DEPTH payload."""
    target_depth_cm: float = 30.0

    _fmt: str = field(default="<f", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.target_depth_cm)

    @classmethod
    def unpack(cls, data: bytes) -> "SetDepth":
        (val,) = struct.unpack(cls._fmt, data)
        return cls(target_depth_cm=val)


@dataclass
class SetYaw:
    """SET_YAW payload."""
    target_yaw_deg: float = 0.0

    _fmt: str = field(default="<f", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.target_yaw_deg)

    @classmethod
    def unpack(cls, data: bytes) -> "SetYaw":
        (val,) = struct.unpack(cls._fmt, data)
        return cls(target_yaw_deg=val)


@dataclass
class SetMotion:
    """SET_MOTION payload."""
    motion_state: int = 0  # 0=STOP, 1=FLOAT, 2=FRONT, 3=BACK, 4=LEFT, 5=RIGHT, 6=CW, 7=CCW

    _fmt: str = field(default="<B", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.motion_state & 0xFF)

    @classmethod
    def unpack(cls, data: bytes) -> "SetMotion":
        (val,) = struct.unpack(cls._fmt, data)
        return cls(motion_state=val)


@dataclass
class StatusReport:
    """STATUS_REPORT payload (24 bytes)."""
    safety_state: int = 0
    flags: int = 0
    pwm: List[int] = field(default_factory=lambda: [1500] * 8)
    error_count: int = 0
    heartbeat_missed: int = 0
    neutral_reason: int = NeutralReason.NONE
    active_channel: int = 0xFF

    _fmt: str = field(default="<BB8hHHBB", repr=False, init=False)

    @property
    def control_enable(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def float_enabled(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def angle_enabled(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def manual_pwm_enabled(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def estop_locked(self) -> bool:
        return bool(self.flags & 0x10)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.safety_state & 0xFF, self.flags & 0xFF,
                           *self.pwm, self.error_count & 0xFFFF,
                           self.heartbeat_missed & 0xFFFF,
                           self.neutral_reason & 0xFF,
                           self.active_channel & 0xFF)

    @classmethod
    def unpack(cls, data: bytes) -> "StatusReport":
        vals = struct.unpack(cls._fmt, data)
        return cls(
            safety_state=vals[0], flags=vals[1],
            pwm=list(vals[2:10]),
            error_count=vals[10], heartbeat_missed=vals[11],
            neutral_reason=vals[12], active_channel=vals[13],
        )


@dataclass
class SensorReport:
    """SENSOR_REPORT payload."""
    depth_m: float = 0.0
    pressure_mbar: float = 0.0
    water_temp_c: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    accel: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mag: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    yaw_v: float = 0.0
    pitch_v: float = 0.0
    roll_v: float = 0.0

    _fmt: str = field(default="<ffffffffffffffffff", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt,
                           self.depth_m, self.pressure_mbar, self.water_temp_c,
                           self.yaw, self.pitch, self.roll,
                           *self.accel, *self.gyro, *self.mag,
                           self.yaw_v, self.pitch_v, self.roll_v)

    @classmethod
    def unpack(cls, data: bytes) -> "SensorReport":
        vals = struct.unpack(cls._fmt, data)
        return cls(
            depth_m=vals[0], pressure_mbar=vals[1], water_temp_c=vals[2],
            yaw=vals[3], pitch=vals[4], roll=vals[5],
            accel=list(vals[6:9]), gyro=list(vals[9:12]), mag=list(vals[12:15]),
            yaw_v=vals[15], pitch_v=vals[16], roll_v=vals[17],
        )


@dataclass
class ProtoAck:
    """ACK payload."""
    ack_seq: int = 0

    _fmt: str = field(default="<H", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.ack_seq & 0xFFFF)

    @classmethod
    def unpack(cls, data: bytes) -> "ProtoAck":
        (val,) = struct.unpack(cls._fmt, data)
        return cls(ack_seq=val)


@dataclass
class ProtoNack:
    """NACK payload."""
    rejected_sequence: int = 0
    original_type: int = 0
    reason: int = 0

    _fmt: str = field(default="<HBB", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.rejected_sequence & 0xFFFF,
                           self.original_type & 0xFF, self.reason & 0xFF)

    @classmethod
    def unpack(cls, data: bytes) -> "ProtoNack":
        rejected_sequence, original_type, reason = struct.unpack(cls._fmt, data)
        return cls(rejected_sequence=rejected_sequence,
                   original_type=original_type, reason=reason)


@dataclass
class SafetyEvent:
    """SAFETY_EVENT payload."""
    event_type: int = 0
    reason_code: int = 0

    _fmt: str = field(default="<BBH", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.event_type & 0xFF, self.reason_code & 0xFF, 0)

    @classmethod
    def unpack(cls, data: bytes) -> "SafetyEvent":
        et, rc, _ = struct.unpack(cls._fmt, data)
        return cls(event_type=et, reason_code=rc)


@dataclass
class Heartbeat:
    """HEARTBEAT payload."""
    heartbeat_timeout_ms: int = 1000

    _fmt: str = field(default="<H", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.heartbeat_timeout_ms & 0xFFFF)

    @classmethod
    def unpack(cls, data: bytes) -> "Heartbeat":
        (val,) = struct.unpack(cls._fmt, data)
        return cls(heartbeat_timeout_ms=val)


@dataclass
class HeartbeatAck:
    """HEARTBEAT_ACK payload."""
    safety_state: int = 0
    uptime_s: int = 0
    error_count: int = 0

    _fmt: str = field(default="<BBHH", repr=False, init=False)

    def pack(self) -> bytes:
        return struct.pack(self._fmt, self.safety_state & 0xFF, 0,
                           self.uptime_s & 0xFFFF, self.error_count & 0xFFFF)

    @classmethod
    def unpack(cls, data: bytes) -> "HeartbeatAck":
        ss, _, up, ec = struct.unpack(cls._fmt, data)
        return cls(safety_state=ss, uptime_s=up, error_count=ec)


# ---------------------------------------------------------------------------
#  Payload packer/unpacker dispatch table
# ---------------------------------------------------------------------------

_PAYLOAD_CLASSES = {
    MsgType.SET_PWM: SetPwm,
    MsgType.SET_DEPTH: SetDepth,
    MsgType.SET_YAW: SetYaw,
    MsgType.SET_MOTION: SetMotion,
    MsgType.STATUS_REPORT: StatusReport,
    MsgType.SENSOR_REPORT: SensorReport,
    MsgType.ACK: ProtoAck,
    MsgType.NACK: ProtoNack,
    MsgType.SAFETY_EVENT: SafetyEvent,
    MsgType.HEARTBEAT: Heartbeat,
    MsgType.HEARTBEAT_ACK: HeartbeatAck,
}


# ---------------------------------------------------------------------------
#  Frame encode / decode
# ---------------------------------------------------------------------------


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """Encode a complete protocol frame (header + payload + CRC).

    Args:
        msg_type: ProtoMsgType value.
        seq: Sequence number (uint16).
        payload: Raw payload bytes (max 240).

    Returns:
        Complete frame bytes.

    Raises:
        ValueError: if payload exceeds MAX_PAYLOAD.
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"Payload {len(payload)} exceeds max {MAX_PAYLOAD}")

    # Header (8 bytes)
    header = struct.pack("<BBBBHH",
                         MAGIC_0, MAGIC_1, VERSION,
                         msg_type & 0xFF,
                         seq & 0xFFFF,
                         len(payload) & 0xFFFF)

    # CRC over header + payload
    crc = crc16(header + payload)

    return header + payload + struct.pack("<H", crc)


def decode_frame(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    """Decode a protocol frame.

    Args:
        data: Raw bytes starting at the magic bytes.

    Returns:
        (msg_type, seq, payload) on success, None on any failure
        (bad magic, bad version, CRC mismatch, truncated).
    """
    if len(data) < FRAME_OVERHEAD:
        return None

    magic0, magic1, version, msg_type, seq, payload_len = \
        struct.unpack_from("<BBBBHH", data, 0)

    if magic0 != MAGIC_0 or magic1 != MAGIC_1:
        return None
    if version != VERSION:
        return None
    if payload_len > MAX_PAYLOAD:
        return None

    total_len = FRAME_OVERHEAD + payload_len + CRC_SIZE
    if len(data) < total_len:
        return None

    # Extract CRC (last 2 bytes before payload end)
    crc_offset = FRAME_OVERHEAD + payload_len
    received_crc = struct.unpack_from("<H", data, crc_offset)[0]

    # Compute expected CRC over header + payload
    expected_crc = crc16(data[:crc_offset])

    if received_crc != expected_crc:
        return None

    payload = data[FRAME_OVERHEAD:crc_offset]
    return (msg_type, seq, payload)


def find_frame_start(data: bytes, start: int = 0) -> int:
    """Find the next occurrence of MAGIC_0 followed by MAGIC_1.

    Returns:
        Index of MAGIC_0, or -1 if not found.
    """
    for i in range(start, len(data) - 1):
        if data[i] == MAGIC_0 and data[i + 1] == MAGIC_1:
            return i
    return -1


def unpack_payload(msg_type: int, payload: bytes):
    """Unpack payload bytes into the appropriate dataclass.

    Returns:
        Dataclass instance, or None if unknown type or insufficient data.
    """
    cls = _PAYLOAD_CLASSES.get(msg_type)
    if cls is None:
        return None
    try:
        return cls.unpack(payload)
    except (struct.error, IndexError):
        return None


def pack_payload(msg_type: int, obj) -> Optional[bytes]:
    """Pack a dataclass object into payload bytes."""
    cls = _PAYLOAD_CLASSES.get(msg_type)
    if cls is None or not isinstance(obj, cls):
        return None
    try:
        return obj.pack()
    except struct.error:
        return None
