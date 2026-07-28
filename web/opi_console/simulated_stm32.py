"""
Simulated STM32 firmware for protocol development and testing.

Implements the full binary protocol state machine, mock sensors,
and configurable fault injection.  Connects via virtual serial (asyncio).
"""

import asyncio
import logging
import math
import random
import struct
import time
from dataclasses import dataclass
from typing import Optional, List

# Protocol imports (relative to project root when run with PYTHONPATH)
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "protocol", "shared"))

from protocol import (
    MAGIC_0, FRAME_OVERHEAD, MAX_PAYLOAD, CRC_SIZE,
    PROTO_BUF_SIZE, MsgType, SafetyState, NackReason, NeutralReason,
    SetPwm, SetDepth, SetBodyCommand, MotionTuning, DepthPidTuning,
    StatusReport, SensorReport, DepthControlReport,
    ProtoAck, ProtoNack,
    SafetyEvent, Heartbeat, HeartbeatAck,
    encode_frame, decode_frame, find_frame_start,
)
from opi_console.motion_tuning import validate_motion_tuning
from opi_console.depth_pid_tuning import validate_depth_pid_tuning

logger = logging.getLogger("opi_console.simulator")

_AXIS_DIRECTIONS = (
    (-1, -1, 0, 0, 0, 1),
    (0, 0, 1, -1, 1, 0),
    (0, 0, 1, 1, -1, 0),
    (-1, 1, 0, 0, 0, 0),
    (1, -1, 0, 0, 0, -1),
    (0, 0, -1, 1, 1, 0),
    (0, 0, -1, -1, -1, 0),
    (1, 1, 0, 0, 0, 0),
)
_HORIZONTAL_CHANNELS = frozenset((0, 3, 4, 7))
_NEUTRAL_TRIM_US = (0, 0, 0, 0, 0, 0, 0, 0)
_DEADZONE_US = (50, 50, 50, 50, 50, 50, 50, 50)


# ============================================================================
#  Mock sensor model
# ============================================================================

@dataclass
class MockSensors:
    """Generates realistic synthetic sensor data."""
    depth_m: float = 1.5          # current depth
    target_depth_m: float = 1.5   # target depth (for PID sim)
    yaw_rad: float = 0.0
    pitch_rad: float = 0.0
    roll_rad: float = 0.0
    # Noise parameters
    depth_noise: float = 0.005
    imu_noise: float = 0.01
    mag_noise: float = 0.5
    # Internal
    _time: float = 0.0
    _depth_base: float = 1.5

    def update(self, dt_s: float, pwm_active: List[int] = None):
        """Advance the sensor model by dt_s seconds."""
        self._time += dt_s

        # Depth: slowly drift toward target + sine wave + noise
        if pwm_active is None:
            pwm_active = [1500] * 8
        vertical_thrust = (
            (pwm_active[1] - 1500)
            + (pwm_active[2] - 1500)
            - (pwm_active[5] - 1500)
            - (pwm_active[6] - 1500)
        ) / 4.0
        depth_rate = vertical_thrust * 0.0001  # m/s per PWM offset unit
        self._depth_base += depth_rate * dt_s
        self.depth_m = (
            self._depth_base
            + 0.05 * math.sin(self._time * 0.3)
            + random.gauss(0, self.depth_noise)
        )

        # Attitude: slow sinusoidal variation + noise
        self.yaw_rad   = 0.1 * math.sin(self._time * 0.1) + random.gauss(0, self.imu_noise * 0.1)
        self.pitch_rad = 0.05 * math.sin(self._time * 0.15) + random.gauss(0, self.imu_noise * 0.05)
        self.roll_rad  = 0.03 * math.sin(self._time * 0.2) + random.gauss(0, self.imu_noise * 0.05)

    def get_accel(self) -> List[float]:
        """Simulated accelerometer (m/s^2)."""
        return [
            random.gauss(0, self.imu_noise),
            random.gauss(0, self.imu_noise),
            9.81 + random.gauss(0, self.imu_noise * 0.3),
        ]

    def get_gyro(self) -> List[float]:
        """Simulated gyroscope (rad/s)."""
        return [
            random.gauss(self.roll_v(), self.imu_noise * 0.2),
            random.gauss(self.pitch_v(), self.imu_noise * 0.2),
            random.gauss(self.yaw_v(), self.imu_noise * 0.2),
        ]

    def get_mag(self) -> List[float]:
        """Simulated magnetometer (uT)."""
        return [
            25.0 + random.gauss(0, self.mag_noise),
            -10.0 + random.gauss(0, self.mag_noise),
            45.0 + random.gauss(0, self.mag_noise),
        ]

    def roll_v(self) -> float:
        return 0.01 * math.cos(self._time * 0.2)
    def pitch_v(self) -> float:
        return 0.01 * math.cos(self._time * 0.15)
    def yaw_v(self) -> float:
        return 0.02 * math.cos(self._time * 0.1)


# ============================================================================
#  Simulated STM32
# ============================================================================

class SimulatedStm32:
    """Full behavioral simulation of the STM32 firmware.

    Implements:
    - Binary protocol parser (identical state machine)
    - Safety state machine (DISARMED → ARMED_IDLE → MANUAL_TEST, etc.)
    - Heartbeat timeout → COMM_LOST
    - Channel timeout → auto-neutral
    - ESTOP latching
    - Configurable fault injection
    """

    def __init__(self,
                 heartbeat_timeout_ms: int = 1000,
                 channel_timeout_ms: int = 500,
                 pwm_neutral: int = 1500,
                 pwm_test_min: int = 1450,
                 pwm_test_max: int = 1550,
                 inject_crc_error_every: int = 0,    # 0 = disabled
                 drop_frames_after_seconds: int = 0,  # 0 = disabled
                 ack_delay_ms: int = 0,
                 drop_acks: bool = False,
                 channel_busy_on_switch: bool = True,
                 status_report_hz: float = 5.0,
                 sensor_report_hz: float = 0.0,
                 min_test_duration_ms: int = 200,
                 max_test_duration_ms: int = 2000,
                 tx_queue_size: int = 256,
                 ):
        self._heartbeat_timeout = heartbeat_timeout_ms / 1000.0
        self._channel_timeout_ms = channel_timeout_ms
        self._pwm_neutral = pwm_neutral
        self._pwm_test_min = pwm_test_min
        self._pwm_test_max = pwm_test_max
        self._inject_crc_every = inject_crc_error_every
        self._drop_after = drop_frames_after_seconds
        self.ack_delay_ms = ack_delay_ms
        self.drop_acks = drop_acks
        self.channel_busy_on_switch = channel_busy_on_switch
        self.nack_next_reason: Optional[int] = None
        self._status_report_interval = 1.0 / status_report_hz if status_report_hz > 0 else 0.0
        self._sensor_report_interval = 1.0 / sensor_report_hz if sensor_report_hz > 0 else 0.0
        self._min_test_duration_ms = min_test_duration_ms
        self._max_test_duration_ms = max_test_duration_ms

        # State
        self.safety_state: SafetyState = SafetyState.DISARMED
        self.control_enable: bool = False
        self.float_enabled: bool = False
        self.angle_enabled: bool = False
        self.body_control_enabled: bool = False
        self.estop_locked: bool = False
        self.active_channel: int = -1  # -1 = none
        self.channel_deadline: float = 0.0
        self.last_heartbeat: float = 0.0
        self.neutral_reason: NeutralReason = NeutralReason.NONE
        self.heartbeat_missed: int = 0
        self.pwm: List[int] = [self._pwm_neutral] * 8
        self._desired_body_pwm: List[int] = [self._pwm_neutral] * 8
        self.body_command = SetBodyCommand()
        self.body_command_last_at: float = 0.0
        self.motion_tuning = MotionTuning()
        self.depth_pid_tuning = DepthPidTuning()
        self.depth_requested_target_cm: float = 150.0
        self.depth_active_setpoint_cm: float = self.depth_requested_target_cm
        self.depth_error_cm: float = 0.0
        self.depth_pid_p_us: float = 0.0
        self.depth_pid_i_us: float = 0.0
        self.depth_pid_d_us: float = 0.0
        self.depth_pid_output_us: float = 0.0
        self.depth_pid_saturated: bool = False
        self.depth_sensor_ready: bool = True
        self.depth_sample_valid: bool = True
        self.depth_sample_updates_enabled: bool = True
        self.actuator_output_ready: bool = True
        self.depth_fault_reason: int = NeutralReason.NONE
        self.depth_command_last_at: float = 0.0
        self._depth_sample_at: float = 0.0
        self._depth_pid_accumulator_s: float = 0.0
        self._depth_integral_error: float = 0.0
        self._depth_previous_error: float = 0.0
        self.horizontal_saturated: bool = False
        self.vertical_saturated: bool = False
        self.seq: int = 0
        self.error_count: int = 0
        self.uptime_s: int = 0
        self._last_uptime: float = 0.0
        self._frame_count: int = 0
        self._tx_frame_count: int = 0
        self._start_time: float = 0.0
        self._next_status_report: float = 0.0
        self._next_sensor_report: float = 0.0
        self._link_connected: bool = True
        self._running: bool = False
        self._delayed_tasks: set[asyncio.Task] = set()
        self.rx_overflow_count: int = 0
        self.tx_dropped_frames: int = 0

        # Sensors
        self.sensors = MockSensors()

        # RX buffer for protocol parsing
        self._rx_buf: bytearray = bytearray()

        # TX queue (frames to send to host)
        self._tx_queue: asyncio.Queue = asyncio.Queue(maxsize=max(8, tx_queue_size))
        self._tx_control_reserve = min(4, self._tx_queue.maxsize // 2)

    # -------- Public API --------

    async def start(self):
        """Called when the simulator begins."""
        now = time.monotonic()
        self._running = True
        self._link_connected = True
        self._start_time = now
        self._last_uptime = now
        self._next_status_report = now + self._status_report_interval if self._status_report_interval else 0.0
        self._next_sensor_report = now + self._sensor_report_interval if self._sensor_report_interval else 0.0
        self.last_heartbeat = self._start_time
        self._depth_sample_at = now
        logger.info("Simulated STM32 started — DISARMED")

    async def stop(self):
        """Called when the simulator shuts down."""
        self._running = False
        for task in list(self._delayed_tasks):
            task.cancel()
        if self._delayed_tasks:
            await asyncio.gather(*self._delayed_tasks, return_exceptions=True)
        self._delayed_tasks.clear()
        logger.info("Simulated STM32 stopped")

    async def restart(self):
        """Simulate an STM32 reset while keeping the virtual serial link."""
        now = time.monotonic()
        self.safety_state = SafetyState.DISARMED
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self.estop_locked = False
        self.active_channel = -1
        self.channel_deadline = 0.0
        self.last_heartbeat = now
        self.heartbeat_missed = 0
        self.pwm = [self._pwm_neutral] * 8
        self._desired_body_pwm = [self._pwm_neutral] * 8
        self.body_command = SetBodyCommand()
        self.body_command_last_at = 0.0
        self.motion_tuning = MotionTuning()
        self.depth_pid_tuning = DepthPidTuning()
        self.depth_requested_target_cm = self.sensors.depth_m * 100.0
        self.depth_active_setpoint_cm = self.depth_requested_target_cm
        self.depth_error_cm = 0.0
        self.depth_pid_p_us = 0.0
        self.depth_pid_i_us = 0.0
        self.depth_pid_d_us = 0.0
        self.depth_pid_output_us = 0.0
        self.depth_pid_saturated = False
        self.depth_sensor_ready = True
        self.depth_sample_valid = True
        self.depth_sample_updates_enabled = True
        self.actuator_output_ready = True
        self.depth_fault_reason = NeutralReason.NONE
        self.depth_command_last_at = 0.0
        self._depth_sample_at = now
        self._depth_pid_accumulator_s = 0.0
        self._depth_integral_error = 0.0
        self._depth_previous_error = 0.0
        self.horizontal_saturated = False
        self.vertical_saturated = False
        self.neutral_reason = NeutralReason.NONE
        self.seq = 0
        self.error_count = 0
        self.uptime_s = 0
        self._start_time = now
        self._last_uptime = now
        self._next_status_report = now + self._status_report_interval if self._status_report_interval else 0.0
        self._next_sensor_report = now + self._sensor_report_interval if self._sensor_report_interval else 0.0
        self._rx_buf.clear()
        for task in list(self._delayed_tasks):
            task.cancel()
        if self._delayed_tasks:
            await asyncio.gather(*self._delayed_tasks, return_exceptions=True)
        self._delayed_tasks.clear()
        while not self._tx_queue.empty():
            try:
                self._tx_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        logger.info("Simulated STM32 restarted — DISARMED")
        if self._link_connected:
            await self._send_status_report(high_priority=True)

    @property
    def connected(self) -> bool:
        return self._link_connected

    async def disconnect(self):
        """Simulate a physical serial link loss."""
        self._link_connected = False

    async def reconnect(self):
        """Restore a previously disconnected virtual serial link."""
        now = time.monotonic()
        for task in list(self._delayed_tasks):
            task.cancel()
        if self._delayed_tasks:
            await asyncio.gather(*self._delayed_tasks, return_exceptions=True)
        self._delayed_tasks.clear()
        while not self._tx_queue.empty():
            try:
                self._tx_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        self._link_connected = True
        self._start_time = now
        self._rx_buf.clear()

    async def feed_bytes(self, data: bytes):
        """Feed raw bytes from the host (serial RX)."""
        if (self._drop_after > 0
                and time.monotonic() - self._start_time > self._drop_after):
            await self.disconnect()
        if not self._link_connected:
            raise ConnectionError("simulated serial link disconnected")
        self._rx_buf.extend(data)
        max_buffer = PROTO_BUF_SIZE * 4
        if len(self._rx_buf) > max_buffer:
            self.rx_overflow_count += 1
            self.error_count += 1
            self._rx_buf = self._rx_buf[-max_buffer:]
        await self._parse_frames()

    async def read_frame(self) -> Optional[bytes]:
        """Read the next outbound frame (non-blocking)."""
        if not self._link_connected:
            raise ConnectionError("simulated serial link disconnected")
        try:
            return self._tx_queue.get_nowait()
        except asyncio.QueueEmpty:
            return None

    async def read_frame_wait(self, timeout: float = 1.0) -> Optional[bytes]:
        """Read the next outbound frame (wait up to timeout)."""
        if not self._link_connected:
            raise ConnectionError("simulated serial link disconnected")
        try:
            return await asyncio.wait_for(self._tx_queue.get(), timeout)
        except asyncio.TimeoutError:
            return None

    async def tick(self, dt_s: float):
        """Advance the simulator by dt_s seconds.

        This should be called periodically (e.g., every 10ms) to update
        sensors, check timeouts, and send periodic telemetry.
        """
        now = time.monotonic()

        # Update sensors
        self.sensors.update(dt_s, self.pwm)
        if (
            self.depth_sensor_ready
            and self.depth_sample_valid
            and self.depth_sample_updates_enabled
        ):
            self._depth_sample_at = now

        # Uptime
        if now - self._last_uptime >= 1.0:
            self._last_uptime = now
            self.uptime_s += 1

        # Heartbeat timeout → COMM_LOST
        if self.safety_state in (SafetyState.ARMED_IDLE, SafetyState.ARMED_ACTIVE, SafetyState.MANUAL_TEST):
            if (now - self.last_heartbeat) > self._heartbeat_timeout:
                await self._enter_comm_lost()

        # Channel test timeout
        if self.active_channel >= 0 and now >= self.channel_deadline:
            await self._cancel_channel_test()

        if self.float_enabled:
            sample_fresh = bool(
                self.depth_sensor_ready
                and self.depth_sample_valid
                and math.isfinite(self.sensors.depth_m)
                and -10.0 <= self.sensors.depth_m <= 300.0
                and self._depth_sample_at > 0.0
                and (now - self._depth_sample_at) <= 0.5
            )
            if not sample_fresh or not self.actuator_output_ready:
                self._disable_depth_control(
                    NeutralReason.DEPTH_SENSOR
                    if not sample_fresh else NeutralReason.FAULT)
            elif (
                self.depth_command_last_at <= 0.0
                or (now - self.depth_command_last_at) > 0.5
            ):
                self._disable_depth_control(NeutralReason.COMMAND)
            else:
                self._apply_depth_control(dt_s)

        if self.body_control_enabled:
            nonzero = any(value != 0.0 for value in self.body_command.values())
            timeout_s = self.motion_tuning.command_timeout_ms / 1000.0
            if (
                nonzero
                and self.body_command_last_at > 0.0
                and (now - self.body_command_last_at) > timeout_s
            ):
                self.body_control_enabled = False
                self.control_enable = False
                self.safety_state = SafetyState.ARMED_IDLE
                self.body_command = SetBodyCommand()
                self._all_neutral(NeutralReason.COMMAND)
            else:
                self._apply_body_slew(dt_s)

        if self._status_report_interval and now >= self._next_status_report:
            self._next_status_report = now + self._status_report_interval
            await self._send_status_report()

        if self._sensor_report_interval and now >= self._next_sensor_report:
            self._next_sensor_report = now + self._sensor_report_interval
            await self._send_sensor_report()

        # Simulate disconnection after N seconds
        if self._drop_after > 0 and (now - self._start_time) > self._drop_after:
            await self.disconnect()

    # -------- Internal: Frame parsing --------

    async def _parse_frames(self):
        """Parse complete frames from the RX buffer."""
        while True:
            idx = find_frame_start(bytes(self._rx_buf))
            if idx < 0:
                self._rx_buf = self._rx_buf[-1:] if self._rx_buf[-1:] == bytes([MAGIC_0]) else bytearray()
                break

            # Discard bytes before sync
            if idx > 0:
                self._rx_buf = self._rx_buf[idx:]

            if len(self._rx_buf) < FRAME_OVERHEAD:
                break

            try:
                payload_len = struct.unpack_from("<H", self._rx_buf, 6)[0]
            except struct.error:
                break

            if payload_len > MAX_PAYLOAD:
                self.error_count += 1
                self._rx_buf = self._rx_buf[2:]
                continue

            frame_len = FRAME_OVERHEAD + payload_len + CRC_SIZE
            if len(self._rx_buf) < frame_len:
                break

            frame = bytes(self._rx_buf[:frame_len])
            result = decode_frame(frame)
            if result is None:
                # CRC failure or incomplete — skip sync bytes, try again
                self.error_count += 1
                self._rx_buf = self._rx_buf[2:]
                continue

            msg_type, seq, payload = result

            # Count accepted host frames; CRC faults are injected on TX so the
            # Orange Pi parser can exercise its recovery path.
            self._frame_count += 1

            # Consume the frame
            self._rx_buf = self._rx_buf[frame_len:]

            # Handle the message
            await self._handle_frame(msg_type, seq, payload)

    async def _handle_frame(self, msg_type: int, seq: int, payload: bytes):
        """Dispatch a received frame to the appropriate handler."""
        handlers = {
            MsgType.HEARTBEAT: self._h_heartbeat,
            MsgType.ARM: self._h_arm,
            MsgType.DISARM: self._h_disarm,
            MsgType.EMERGENCY_STOP: self._h_estop,
            MsgType.RESET_ESTOP: self._h_reset_estop,
            MsgType.ENTER_MANUAL: self._h_enter_manual,
            MsgType.EXIT_MANUAL: self._h_exit_manual,
            MsgType.SET_PWM: self._h_set_pwm,
            MsgType.SET_ALL_NEUTRAL: self._h_set_all_neutral,
            MsgType.FLOAT_ON: self._h_float_on,
            MsgType.FLOAT_OFF: self._h_float_off,
            MsgType.SET_DEPTH: self._h_set_depth,
            MsgType.SET_BODY_COMMAND: self._h_set_body_command,
            MsgType.BODY_CONTROL_ON: self._h_body_control_on,
            MsgType.BODY_CONTROL_OFF: self._h_body_control_off,
            MsgType.SET_MOTION_TUNING: self._h_set_motion_tuning,
            MsgType.SET_DEPTH_PID_TUNING: self._h_set_depth_pid_tuning,
            MsgType.REQUEST_STATUS: self._h_request_status,
            MsgType.REQUEST_SENSORS: self._h_request_sensors,
            MsgType.REQUEST_MOTION_TUNING: self._h_request_motion_tuning,
            MsgType.REQUEST_DEPTH_PID_TUNING:
                self._h_request_depth_pid_tuning,
            MsgType.REQUEST_DEPTH_CONTROL: self._h_request_depth_control,
        }
        handler = handlers.get(msg_type)
        if handler is None:
            await self._send_nack(seq, msg_type, NackReason.UNSUPPORTED_MESSAGE)
            return

        expected_lengths = {
            MsgType.HEARTBEAT: 2,
            MsgType.ARM: 0,
            MsgType.DISARM: 0,
            MsgType.EMERGENCY_STOP: 0,
            MsgType.RESET_ESTOP: 0,
            MsgType.ENTER_MANUAL: 0,
            MsgType.EXIT_MANUAL: 0,
            MsgType.SET_PWM: 6,
            MsgType.SET_ALL_NEUTRAL: 0,
            MsgType.FLOAT_ON: 0,
            MsgType.FLOAT_OFF: 0,
            MsgType.SET_DEPTH: 4,
            MsgType.SET_BODY_COMMAND: 24,
            MsgType.BODY_CONTROL_ON: 0,
            MsgType.BODY_CONTROL_OFF: 0,
            MsgType.SET_MOTION_TUNING: 56,
            MsgType.SET_DEPTH_PID_TUNING: 28,
            MsgType.REQUEST_STATUS: 0,
            MsgType.REQUEST_SENSORS: 0,
            MsgType.REQUEST_MOTION_TUNING: 0,
            MsgType.REQUEST_DEPTH_PID_TUNING: 0,
            MsgType.REQUEST_DEPTH_CONTROL: 0,
        }
        if len(payload) != expected_lengths[msg_type]:
            self.error_count += 1
            if msg_type == MsgType.SET_BODY_COMMAND:
                self.body_control_enabled = False
                self.control_enable = False
                self.body_command = SetBodyCommand()
                self.body_command_last_at = 0.0
                if (
                    not self.estop_locked
                    and self.safety_state == SafetyState.ARMED_ACTIVE
                ):
                    self.safety_state = SafetyState.ARMED_IDLE
                self._all_neutral(
                    NeutralReason.EMERGENCY_STOP
                    if self.estop_locked else NeutralReason.COMMAND)
            await self._send_nack(
                seq, msg_type, NackReason.INVALID_PAYLOAD_LENGTH)
            return
        await handler(seq, payload)

    # -------- Internal: State transitions --------

    async def _enter_comm_lost(self):
        if self.safety_state == SafetyState.COMM_LOST:
            return
        logger.warning("Simulator: heartbeat timeout → COMM_LOST")
        self.safety_state = SafetyState.COMM_LOST
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self.heartbeat_missed += 1
        self._all_neutral(NeutralReason.COMM_LOST)
        await self._send_status_report(high_priority=True)
        event = SafetyEvent(event_type=0, reason_code=NeutralReason.COMM_LOST)
        await self._send_frame(MsgType.SAFETY_EVENT, event.pack(), high_priority=True)

    async def _cancel_channel_test(self):
        logger.debug("Simulator: channel %d timeout → neutral", self.active_channel)
        if 0 <= self.active_channel < 8:
            self.pwm[self.active_channel] = self._pwm_neutral
        self.active_channel = -1
        self.channel_deadline = 0.0
        self.neutral_reason = NeutralReason.PWM_COMMAND_TIMEOUT
        await self._send_status_report(high_priority=True)

    def _all_neutral(self, reason: NeutralReason = NeutralReason.COMMAND):
        for i in range(8):
            self.pwm[i] = self._pwm_neutral
            self._desired_body_pwm[i] = self._pwm_neutral
        self.active_channel = -1
        self.channel_deadline = 0.0
        self.horizontal_saturated = False
        self.vertical_saturated = False
        self.neutral_reason = reason
        if not self.float_enabled:
            self.depth_command_last_at = 0.0
            self.depth_fault_reason = int(reason)
            self._reset_depth_pid_runtime()

    def _reset_depth_pid_runtime(self):
        self.depth_error_cm = 0.0
        self.depth_pid_p_us = 0.0
        self.depth_pid_i_us = 0.0
        self.depth_pid_d_us = 0.0
        self.depth_pid_output_us = 0.0
        self.depth_pid_saturated = False
        self._depth_pid_accumulator_s = 0.0
        self._depth_integral_error = 0.0
        self._depth_previous_error = 0.0

    def _disable_depth_control(self, reason: NeutralReason):
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        if (
            not self.estop_locked
            and self.safety_state == SafetyState.ARMED_ACTIVE
        ):
            self.safety_state = SafetyState.ARMED_IDLE
        self.depth_command_last_at = 0.0
        self.depth_fault_reason = int(reason)
        self._reset_depth_pid_runtime()
        self._all_neutral(reason)

    def _apply_depth_control(self, dt_s: float):
        self.depth_active_setpoint_cm = self.depth_requested_target_cm
        measured_cm = self.sensors.depth_m * 100.0
        self.depth_error_cm = (
            self.depth_active_setpoint_cm - measured_cm)
        self._depth_pid_accumulator_s += max(0.0, dt_s)
        if self._depth_pid_accumulator_s >= 0.1:
            self._depth_pid_accumulator_s %= 0.1
            tuning = self.depth_pid_tuning
            raw_p = self.depth_error_cm * tuning.kp
            self.depth_pid_p_us = max(
                -tuning.p_limit_us,
                min(tuning.p_limit_us, raw_p),
            )
            if tuning.ki != 0.0 and tuning.i_limit_us > 0.0:
                self._depth_integral_error += self.depth_error_cm
                integral_error_limit = (
                    tuning.i_limit_us / abs(tuning.ki))
                self._depth_integral_error = max(
                    -integral_error_limit,
                    min(integral_error_limit, self._depth_integral_error),
                )
            else:
                self._depth_integral_error = 0.0
            self.depth_pid_i_us = max(
                -tuning.i_limit_us,
                min(
                    tuning.i_limit_us,
                    self._depth_integral_error * tuning.ki,
                ),
            )
            raw_d = (
                (self.depth_error_cm - self._depth_previous_error)
                * tuning.kd
            )
            self.depth_pid_d_us = max(
                -tuning.d_limit_us,
                min(tuning.d_limit_us, raw_d),
            )
            raw_output = (
                self.depth_pid_p_us
                + self.depth_pid_i_us
                + self.depth_pid_d_us
            )
            self.depth_pid_output_us = max(
                -tuning.output_limit_us,
                min(tuning.output_limit_us, raw_output),
            )
            self.depth_pid_saturated = (
                abs(self.depth_pid_output_us)
                >= tuning.output_limit_us - 0.001
            )
            self._depth_previous_error = self.depth_error_cm

        normalized = self.depth_pid_output_us / 350.0
        scaled = normalized * self.motion_tuning.axis_gain[2]
        limited = max(
            -self.motion_tuning.axis_max_output[2],
            min(self.motion_tuning.axis_max_output[2], scaled),
        )
        self.vertical_saturated = limited != scaled
        applied = limited * self.motion_tuning.global_multiplier * 350.0
        delta = (
            math.floor(applied + 0.5)
            if applied >= 0.0
            else math.ceil(applied - 0.5)
        )
        for channel, direction in ((1, 1), (2, 1), (5, -1), (6, -1)):
            channel_delta = direction * delta
            pwm = self._pwm_neutral + channel_delta
            if channel_delta > 0:
                pwm += _DEADZONE_US[channel]
            elif channel_delta < 0:
                pwm -= _DEADZONE_US[channel]
            self.pwm[channel] = max(1000, min(2000, pwm))
        for channel in _HORIZONTAL_CHANNELS:
            self.pwm[channel] = self._pwm_neutral
        self.neutral_reason = NeutralReason.NONE

    async def _send_frame(self, msg_type: int, payload: bytes = b"",
                          high_priority: bool = False) -> bool:
        if not self._link_connected:
            return False
        frame = encode_frame(msg_type, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFFFF
        self._tx_frame_count += 1
        # Inject CRC error if configured
        if (self._inject_crc_every > 0
                and self._tx_frame_count % self._inject_crc_every == 0):
            frame = bytearray(frame)
            if len(frame) > 2:
                frame[-1] ^= 0xFF
            frame = bytes(frame)
        telemetry_limit = self._tx_queue.maxsize - self._tx_control_reserve
        if not high_priority and self._tx_queue.qsize() >= telemetry_limit:
            self.tx_dropped_frames += 1
            return False
        if self._tx_queue.full():
            self.tx_dropped_frames += 1
            return False
        self._tx_queue.put_nowait(frame)
        return True

    def _track_delayed_task(self, task: asyncio.Task):
        self._delayed_tasks.add(task)
        task.add_done_callback(self._delayed_tasks.discard)

    async def _delayed_send(self, msg_type: int, payload: bytes,
                            delay_s: float, high_priority: bool):
        await asyncio.sleep(delay_s)
        await self._send_frame(msg_type, payload, high_priority=high_priority)

    async def _send_ack(self, seq: int):
        if self.drop_acks:
            logger.debug("Simulator: dropping ACK seq=%d", seq)
            return
        ack = ProtoAck(ack_seq=seq)
        if self.ack_delay_ms > 0:
            task = asyncio.create_task(
                self._delayed_send(MsgType.ACK, ack.pack(),
                                   self.ack_delay_ms / 1000.0, True),
                name=f"sim-delayed-ack-{seq}")
            self._track_delayed_task(task)
            return
        await self._send_frame(MsgType.ACK, ack.pack(), high_priority=True)

    async def _send_nack(self, seq: int, original_type: int, reason: int):
        nack = ProtoNack(rejected_sequence=seq,
                         original_type=original_type, reason=reason)
        await self._send_frame(MsgType.NACK, nack.pack(), high_priority=True)

    def _calculate_body_pwm(self, command: SetBodyCommand) -> List[int]:
        axes = command.values()
        limited_axes = []
        axis_limited = [False] * 6
        for index, value in enumerate(axes):
            scaled = value * self.motion_tuning.axis_gain[index]
            limited = max(
                -self.motion_tuning.axis_max_output[index],
                min(self.motion_tuning.axis_max_output[index], scaled),
            )
            axis_limited[index] = limited != scaled
            limited_axes.append(
                limited * self.motion_tuning.global_multiplier)

        if all(value == 0.0 for value in limited_axes):
            self.horizontal_saturated = any(
                axis_limited[index] for index in (0, 1, 5))
            self.vertical_saturated = any(
                axis_limited[index] for index in (2, 3, 4))
            return [self._pwm_neutral] * 8

        raw = [
            sum(
                direction * limited_axes[axis]
                for axis, direction in enumerate(_AXIS_DIRECTIONS[channel])
            )
            for channel in range(8)
        ]
        horizontal_max = max(
            abs(raw[channel]) for channel in _HORIZONTAL_CHANNELS)
        vertical_max = max(
            abs(raw[channel])
            for channel in range(8)
            if channel not in _HORIZONTAL_CHANNELS
        )
        horizontal_group_limited = horizontal_max > 1.0
        vertical_group_limited = vertical_max > 1.0
        horizontal_scale = (
            1.0 / horizontal_max if horizontal_group_limited else 1.0)
        vertical_scale = (
            1.0 / vertical_max if vertical_group_limited else 1.0)
        self.horizontal_saturated = (
            horizontal_group_limited
            or any(axis_limited[index] for index in (0, 1, 5))
        )
        self.vertical_saturated = (
            vertical_group_limited
            or any(axis_limited[index] for index in (2, 3, 4))
        )

        output: List[int] = []
        for channel in range(8):
            horizontal = channel in _HORIZONTAL_CHANNELS
            normalized = raw[channel] * (
                horizontal_scale if horizontal else vertical_scale)
            span = 450.0 if horizontal else 350.0
            delta_float = normalized * span
            delta = (
                math.floor(delta_float + 0.5)
                if delta_float >= 0.0
                else math.ceil(delta_float - 0.5)
            )
            pwm = self._pwm_neutral
            if delta != 0:
                pwm += _NEUTRAL_TRIM_US[channel] + delta
                if pwm > self._pwm_neutral:
                    pwm += _DEADZONE_US[channel]
                elif pwm < self._pwm_neutral:
                    pwm -= _DEADZONE_US[channel]
            output.append(max(1000, min(2000, pwm)))
        return output

    def _apply_body_slew(self, dt_s: float):
        max_step = max(
            1,
            math.ceil(
                self.motion_tuning.pwm_slew_rate_us_per_s
                * max(0.0, min(dt_s, 0.1))
            ),
        )
        for channel, desired in enumerate(self._desired_body_pwm):
            current = self.pwm[channel]
            if desired > current:
                self.pwm[channel] = min(desired, current + max_step)
            elif desired < current:
                self.pwm[channel] = max(desired, current - max_step)

    # -------- Command handlers --------

    async def _h_heartbeat(self, seq: int, payload: bytes):
        self.last_heartbeat = time.monotonic()
        hb = Heartbeat.unpack(payload)
        if hb.heartbeat_timeout_ms == 0:
            await self._send_nack(seq, MsgType.HEARTBEAT,
                                  NackReason.INVALID_VALUE)
            return
        self._heartbeat_timeout = hb.heartbeat_timeout_ms / 1000.0

        # Recover from COMM_LOST on heartbeat
        if self.safety_state == SafetyState.COMM_LOST:
            logger.info("Simulator: heartbeat restored → DISARMED")
            self.safety_state = SafetyState.DISARMED
            self.control_enable = False
            self.float_enabled = False
            self.angle_enabled = False
            self.body_control_enabled = False
            self._all_neutral(NeutralReason.COMM_LOST)

        # Send heartbeat ACK
        ack = HeartbeatAck(
            safety_state=self.safety_state,
            uptime_s=self.uptime_s,
            error_count=self.error_count,
        )
        await self._send_frame(MsgType.HEARTBEAT_ACK, ack.pack())

    async def _h_arm(self, seq: int, payload: bytes):
        if self.estop_locked or self.safety_state == SafetyState.EMERGENCY_STOP:
            await self._send_nack(seq, MsgType.ARM, NackReason.ESTOP_LOCKED)
            return
        if self.safety_state in (SafetyState.DISARMED, SafetyState.COMM_LOST):
            self.safety_state = SafetyState.ARMED_IDLE
            self.control_enable = False
            self.float_enabled = False
            self.angle_enabled = False
            self.body_control_enabled = False
            self._all_neutral(NeutralReason.NONE)
            await self._send_ack(seq)
            logger.info("Simulator: ARMED_IDLE")
        else:
            await self._send_nack(seq, MsgType.ARM, NackReason.BAD_STATE)

    async def _h_disarm(self, seq: int, payload: bytes):
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        if self.estop_locked:
            self.safety_state = SafetyState.EMERGENCY_STOP
            self._all_neutral(NeutralReason.EMERGENCY_STOP)
        else:
            self.safety_state = SafetyState.DISARMED
            self._all_neutral(NeutralReason.DISARM)
        await self._send_ack(seq)
        logger.info("Simulator: DISARM result=%s", self.safety_state.name)

    async def _h_estop(self, seq: int, payload: bytes):
        self.safety_state = SafetyState.EMERGENCY_STOP
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self.estop_locked = True
        self._all_neutral(NeutralReason.EMERGENCY_STOP)
        await self._send_ack(seq)
        logger.critical("Simulator: EMERGENCY_STOP")

    async def _h_reset_estop(self, seq: int, payload: bytes):
        if self.safety_state != SafetyState.EMERGENCY_STOP:
            await self._send_nack(seq, MsgType.RESET_ESTOP, NackReason.BAD_STATE)
            return
        self.estop_locked = False
        self.safety_state = SafetyState.DISARMED
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self._all_neutral(NeutralReason.DISARM)
        await self._send_ack(seq)
        logger.info("Simulator: ESTOP reset → DISARMED")

    async def _h_enter_manual(self, seq: int, payload: bytes):
        if self.safety_state != SafetyState.ARMED_IDLE:
            await self._send_nack(seq, MsgType.ENTER_MANUAL, NackReason.BAD_STATE)
            return
        self.safety_state = SafetyState.MANUAL_TEST
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self._all_neutral(NeutralReason.COMMAND)
        await self._send_ack(seq)
        logger.info("Simulator: MANUAL_TEST")

    async def _h_exit_manual(self, seq: int, payload: bytes):
        if self.safety_state != SafetyState.MANUAL_TEST:
            await self._send_nack(seq, MsgType.EXIT_MANUAL, NackReason.BAD_STATE)
            return
        self.safety_state = SafetyState.ARMED_IDLE
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self._all_neutral(NeutralReason.COMMAND)
        await self._send_ack(seq)

    async def _h_set_pwm(self, seq: int, payload: bytes):
        if self.nack_next_reason is not None:
            reason = self.nack_next_reason
            self.nack_next_reason = None
            await self._send_nack(seq, MsgType.SET_PWM, reason)
            return

        if self.safety_state != SafetyState.MANUAL_TEST:
            await self._send_nack(seq, MsgType.SET_PWM, NackReason.NOT_ARMED)
            return

        cmd = SetPwm.unpack(payload)
        if payload[1] != 0:
            await self._send_nack(seq, MsgType.SET_PWM,
                                  NackReason.INVALID_VALUE)
            return
        if cmd.channel >= 8:
            await self._send_nack(seq, MsgType.SET_PWM, NackReason.BAD_CHANNEL)
            return
        if cmd.pwm_us < self._pwm_test_min or cmd.pwm_us > self._pwm_test_max:
            await self._send_nack(seq, MsgType.SET_PWM, NackReason.BAD_PWM_VALUE)
            return

        # Single-channel: reset previous channel if different
        if self.active_channel >= 0 and self.active_channel != cmd.channel:
            if self.channel_busy_on_switch:
                await self._send_nack(seq, MsgType.SET_PWM, NackReason.CHANNEL_BUSY)
                return
            self.pwm[self.active_channel] = self._pwm_neutral

        timeout = cmd.timeout_ms if cmd.timeout_ms > 0 else self._channel_timeout_ms
        if (timeout < self._min_test_duration_ms
                or timeout > self._max_test_duration_ms):
            await self._send_nack(seq, MsgType.SET_PWM,
                                  NackReason.INVALID_VALUE)
            return

        self.active_channel = cmd.channel
        self.channel_deadline = time.monotonic() + timeout / 1000.0
        self.pwm[cmd.channel] = cmd.pwm_us
        self.neutral_reason = NeutralReason.NONE

        await self._send_ack(seq)
        logger.debug("Simulator: CH%d = %dus for %dms",
                     cmd.channel, cmd.pwm_us, timeout)

    async def _h_set_all_neutral(self, seq: int, payload: bytes):
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = False
        self.body_command = SetBodyCommand()
        self.body_command_last_at = 0.0
        self._all_neutral(
            NeutralReason.EMERGENCY_STOP
            if self.estop_locked else NeutralReason.COMMAND)
        if (not self.estop_locked
                and self.safety_state in (
                    SafetyState.MANUAL_TEST, SafetyState.ARMED_ACTIVE)):
            self.safety_state = SafetyState.ARMED_IDLE
        await self._send_ack(seq)

    async def _h_float_on(self, seq: int, payload: bytes):
        now = time.monotonic()
        captured_cm = self.sensors.depth_m * 100.0
        depth_ready = bool(
            self.depth_sensor_ready
            and self.depth_sample_valid
            and math.isfinite(captured_cm)
            and 0.0 <= captured_cm <= 30000.0
            and self._depth_sample_at > 0.0
            and (now - self._depth_sample_at) <= 0.5
        )
        if (
            self.estop_locked
            or self.safety_state != SafetyState.ARMED_IDLE
            or not depth_ready
            or not self.actuator_output_ready
        ):
            reason = (
                NackReason.ESTOP_LOCKED
                if self.estop_locked
                else NackReason.BAD_STATE
            )
            await self._send_nack(seq, MsgType.FLOAT_ON, reason)
            return
        self.depth_requested_target_cm = captured_cm
        self.depth_active_setpoint_cm = self.depth_requested_target_cm
        self._reset_depth_pid_runtime()
        self.depth_fault_reason = NeutralReason.NONE
        self.depth_command_last_at = now
        self.safety_state = SafetyState.ARMED_ACTIVE
        self.control_enable = True
        self.float_enabled = True
        self.angle_enabled = False
        self.body_control_enabled = False
        self.body_command = SetBodyCommand()
        self.body_command_last_at = 0.0
        self._all_neutral(NeutralReason.NONE)
        await self._send_ack(seq)

    async def _h_float_off(self, seq: int, payload: bytes):
        if (
            self.safety_state != SafetyState.ARMED_ACTIVE
            or not self.float_enabled
        ):
            await self._send_nack(
                seq, MsgType.FLOAT_OFF, NackReason.BAD_STATE)
            return
        self._disable_depth_control(NeutralReason.COMMAND)
        await self._send_ack(seq)

    async def _h_set_depth(self, seq: int, payload: bytes):
        try:
            command = SetDepth.unpack(payload)
        except struct.error:
            await self._send_nack(
                seq, MsgType.SET_DEPTH,
                NackReason.INVALID_PAYLOAD_LENGTH)
            return
        if (
            not math.isfinite(command.target_depth_cm)
            or command.target_depth_cm < 0.0
            or command.target_depth_cm > 30000.0
        ):
            await self._send_nack(
                seq, MsgType.SET_DEPTH, NackReason.INVALID_VALUE)
            return
        now = time.monotonic()
        depth_ready = bool(
            self.depth_sensor_ready
            and self.depth_sample_valid
            and math.isfinite(self.sensors.depth_m)
            and -10.0 <= self.sensors.depth_m <= 300.0
            and self._depth_sample_at > 0.0
            and (now - self._depth_sample_at) <= 0.5
        )
        if (
            self.estop_locked
            or self.safety_state != SafetyState.ARMED_ACTIVE
            or not self.float_enabled
            or not self.control_enable
            or not depth_ready
            or not self.actuator_output_ready
        ):
            await self._send_nack(
                seq, MsgType.SET_DEPTH, NackReason.BAD_STATE)
            return
        self.depth_requested_target_cm = command.target_depth_cm
        self.depth_command_last_at = now
        self.neutral_reason = NeutralReason.NONE
        await self._send_ack(seq)

    async def _h_body_control_on(self, seq: int, payload: bytes):
        if (
            self.estop_locked
            or self.safety_state != SafetyState.ARMED_IDLE
        ):
            reason = (
                NackReason.ESTOP_LOCKED
                if self.estop_locked
                else NackReason.BAD_STATE
            )
            await self._send_nack(seq, MsgType.BODY_CONTROL_ON, reason)
            return
        self.safety_state = SafetyState.ARMED_ACTIVE
        self.control_enable = True
        self.float_enabled = False
        self.angle_enabled = False
        self.body_control_enabled = True
        self.body_command = SetBodyCommand()
        self.body_command_last_at = time.monotonic()
        self._all_neutral(NeutralReason.NONE)
        await self._send_ack(seq)

    async def _h_body_control_off(self, seq: int, payload: bytes):
        if (
            self.safety_state != SafetyState.ARMED_ACTIVE
            or not self.body_control_enabled
        ):
            await self._send_nack(
                seq, MsgType.BODY_CONTROL_OFF, NackReason.BAD_STATE)
            return
        self.body_control_enabled = False
        self.control_enable = False
        self.float_enabled = False
        self.angle_enabled = False
        self.safety_state = SafetyState.ARMED_IDLE
        self.body_command = SetBodyCommand()
        self.body_command_last_at = 0.0
        self._all_neutral(NeutralReason.COMMAND)
        await self._send_ack(seq)

    async def _h_set_body_command(self, seq: int, payload: bytes):
        try:
            command = SetBodyCommand.unpack(payload)
        except struct.error:
            await self._send_nack(
                seq, MsgType.SET_BODY_COMMAND,
                NackReason.INVALID_PAYLOAD_LENGTH)
            return
        if (
            self.safety_state != SafetyState.ARMED_ACTIVE
            or not self.body_control_enabled
            or not self.control_enable
            or self.estop_locked
        ):
            self.body_control_enabled = False
            self.control_enable = False
            if (
                not self.estop_locked
                and self.safety_state == SafetyState.ARMED_ACTIVE
            ):
                self.safety_state = SafetyState.ARMED_IDLE
            self._all_neutral(
                NeutralReason.EMERGENCY_STOP
                if self.estop_locked
                else NeutralReason.COMMAND)
            await self._send_nack(
                seq, MsgType.SET_BODY_COMMAND, NackReason.BAD_STATE)
            return
        if any(
            not math.isfinite(value) or value < -1.0 or value > 1.0
            for value in command.values()
        ):
            self.body_control_enabled = False
            self.control_enable = False
            self.safety_state = SafetyState.ARMED_IDLE
            self.body_command = SetBodyCommand()
            self._all_neutral(NeutralReason.COMMAND)
            await self._send_nack(
                seq, MsgType.SET_BODY_COMMAND, NackReason.INVALID_VALUE)
            return
        self.body_command = command
        self.body_command_last_at = time.monotonic()
        self._desired_body_pwm = self._calculate_body_pwm(command)
        self.neutral_reason = NeutralReason.NONE
        await self._send_ack(seq)

    async def _h_set_motion_tuning(self, seq: int, payload: bytes):
        try:
            tuning = MotionTuning.unpack(payload)
            validate_motion_tuning(tuning)
        except (struct.error, ValueError, OverflowError):
            await self._send_nack(
                seq, MsgType.SET_MOTION_TUNING, NackReason.INVALID_VALUE)
            return
        if self.estop_locked:
            await self._send_nack(
                seq, MsgType.SET_MOTION_TUNING, NackReason.ESTOP_LOCKED)
            return
        if self.safety_state not in (
            SafetyState.DISARMED,
            SafetyState.ARMED_IDLE,
        ):
            await self._send_nack(
                seq, MsgType.SET_MOTION_TUNING, NackReason.BAD_STATE)
            return
        self.motion_tuning = tuning
        await self._send_ack(seq)

    async def _h_request_motion_tuning(self, seq: int, payload: bytes):
        await self._send_frame(
            MsgType.MOTION_TUNING_REPORT,
            self.motion_tuning.pack(),
            high_priority=True,
        )

    async def _h_set_depth_pid_tuning(self, seq: int, payload: bytes):
        try:
            tuning = DepthPidTuning.unpack(payload)
            validate_depth_pid_tuning(tuning)
        except (struct.error, ValueError, OverflowError):
            await self._send_nack(
                seq, MsgType.SET_DEPTH_PID_TUNING,
                NackReason.INVALID_VALUE)
            return
        if self.estop_locked:
            await self._send_nack(
                seq, MsgType.SET_DEPTH_PID_TUNING,
                NackReason.ESTOP_LOCKED)
            return
        if self.safety_state not in (
            SafetyState.DISARMED,
            SafetyState.ARMED_IDLE,
        ):
            await self._send_nack(
                seq, MsgType.SET_DEPTH_PID_TUNING,
                NackReason.BAD_STATE)
            return
        self.depth_pid_tuning = tuning
        self._reset_depth_pid_runtime()
        await self._send_ack(seq)

    async def _h_request_depth_pid_tuning(
        self, seq: int, payload: bytes
    ):
        await self._send_frame(
            MsgType.DEPTH_PID_TUNING_REPORT,
            self.depth_pid_tuning.pack(),
            high_priority=True,
        )

    async def _h_request_depth_control(self, seq: int, payload: bytes):
        await self._send_depth_control_report(high_priority=True)

    async def _send_depth_control_report(
        self, high_priority: bool = False
    ):
        now = time.monotonic()
        sample_age_ms = (
            min(
                0xFFFFFFFF,
                max(0, int((now - self._depth_sample_at) * 1000)),
            )
            if self._depth_sample_at > 0.0
            else 0xFFFFFFFF
        )
        sample_fresh_valid = bool(
            self.depth_sensor_ready
            and self.depth_sample_valid
            and math.isfinite(self.sensors.depth_m)
            and -10.0 <= self.sensors.depth_m <= 300.0
            and sample_age_ms <= 500
        )
        flags = 0
        if self.float_enabled and self.control_enable:
            flags |= 0x01
        if self.depth_sensor_ready:
            flags |= 0x02
        if sample_fresh_valid:
            flags |= 0x04
        if self.depth_pid_saturated:
            flags |= 0x08
        if self.vertical_saturated:
            flags |= 0x10
        if self.actuator_output_ready:
            flags |= 0x20
        report = DepthControlReport(
            requested_target_cm=self.depth_requested_target_cm,
            active_setpoint_cm=self.depth_active_setpoint_cm,
            measured_depth_cm=self.sensors.depth_m * 100.0,
            error_cm=self.depth_error_cm,
            p_term_us=self.depth_pid_p_us,
            i_term_us=self.depth_pid_i_us,
            d_term_us=self.depth_pid_d_us,
            output_us=self.depth_pid_output_us,
            sample_age_ms=sample_age_ms,
            flags=flags,
            fault_reason=int(self.depth_fault_reason),
            reserved=0,
        )
        await self._send_frame(
            MsgType.DEPTH_CONTROL_REPORT,
            report.pack(),
            high_priority=high_priority,
        )

    async def _h_request_status(self, seq: int, payload: bytes):
        await self._send_status_report(high_priority=True)

    async def _send_status_report(self, high_priority: bool = False):
        report = StatusReport(
            safety_state=self.safety_state,
            flags=self._status_flags(),
            pwm=list(self.pwm),
            error_count=self.error_count,
            heartbeat_missed=self.heartbeat_missed,
            neutral_reason=self.neutral_reason,
            active_channel=self.active_channel if self.active_channel >= 0 else 0xFF,
        )
        await self._send_frame(MsgType.STATUS_REPORT, report.pack(),
                               high_priority=high_priority)

    def _status_flags(self) -> int:
        flags = 0
        if self.control_enable:
            flags |= 0x01
        if self.float_enabled:
            flags |= 0x02
        if self.angle_enabled:
            flags |= 0x04
        if self.safety_state == SafetyState.MANUAL_TEST and self.active_channel >= 0:
            flags |= 0x08  # manual_pwm_enabled
        if self.estop_locked:
            flags |= 0x10
        if self.body_control_enabled:
            flags |= 0x20
        if self.horizontal_saturated:
            flags |= 0x40
        if self.vertical_saturated:
            flags |= 0x80
        return flags

    async def _h_request_sensors(self, seq: int, payload: bytes):
        await self._send_sensor_report(high_priority=True)

    async def _send_sensor_report(self, high_priority: bool = False):
        r = SensorReport(
            depth_m=self.sensors.depth_m,
            pressure_mbar=1013.25 + self.sensors.depth_m * 100.0 * 0.01,
            water_temp_c=22.0,
            yaw=self.sensors.yaw_rad,
            pitch=self.sensors.pitch_rad,
            roll=self.sensors.roll_rad,
            accel=self.sensors.get_accel(),
            gyro=self.sensors.get_gyro(),
            mag=self.sensors.get_mag(),
            yaw_v=self.sensors.yaw_v(),
            pitch_v=self.sensors.pitch_v(),
            roll_v=self.sensors.roll_v(),
        )
        await self._send_frame(MsgType.SENSOR_REPORT, r.pack(),
                               high_priority=high_priority)
