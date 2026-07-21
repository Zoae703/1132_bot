"""
Async serial transport layer for the binary protocol.

Handles: open/close, read/write tasks, heartbeat, auto-reconnect,
frame encode/decode, CRC, and sequence numbering.
"""

import asyncio
import logging
import struct
import time
from enum import Enum
from typing import Optional, Callable, Awaitable, Dict, Any

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "protocol", "shared"))

from protocol import (
    MAGIC_0, FRAME_OVERHEAD, MAX_PAYLOAD, CRC_SIZE, PROTO_BUF_SIZE,
    MsgType,
    encode_frame, decode_frame, find_frame_start, crc16,
    Heartbeat, ProtoAck, ProtoNack,
)

logger = logging.getLogger("opi_console.serial")


class ConnectionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    DISCONNECTING = "DISCONNECTING"


class SerialTransport:
    """Async serial I/O with protocol framing, heartbeat, and auto-reconnect.

    On real hardware: wraps a serial port (via pyserial-asyncio or similar).
    In simulation mode: connects to SimulatedStm32 via asyncio Queue.
    """

    def __init__(self,
                 port: str = "/dev/ttyS5",
                 baudrate: int = 115200,
                 heartbeat_interval: float = 0.2,
                 heartbeat_timeout_ms: int = 1000,
                 reconnect_delays: tuple = (1, 2, 4, 8, 15, 30),
                 max_reconnect_retries: int = 0,
                 config: Optional[Dict[str, Any]] = None):
        self._port = port
        self._baudrate = baudrate
        self._heartbeat_interval = heartbeat_interval
        self._heartbeat_timeout_ms = heartbeat_timeout_ms
        self._reconnect_delays = reconnect_delays
        self._max_reconnect_retries = max_reconnect_retries
        self._config = config or {}

        # State
        self._connected: bool = False
        self._connection_state = ConnectionState.DISCONNECTED
        self.connection_generation: int = 0
        self._seq: int = 0
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._sim_stm32: Optional[Any] = None  # SimulatedStm32 instance
        self._running: bool = False
        self._reader_task_handle: Optional[asyncio.Task] = None
        self._heartbeat_task_handle: Optional[asyncio.Task] = None
        self._reconnect_task_handle: Optional[asyncio.Task] = None
        self._serial_reader_task_handle: Optional[asyncio.Task] = None
        self._sim_tick_task: Optional[asyncio.Task] = None
        self._write_lock = asyncio.Lock()
        self._connection_lock = asyncio.Lock()
        self._disconnect_lock = asyncio.Lock()
        self._callback_tasks: set[asyncio.Task] = set()

        # RX buffer
        self._rx_buf: bytearray = bytearray()

        # Callbacks
        self._frame_callbacks: list = []

        # Stats
        self.tx_frames: int = 0
        self.rx_frames: int = 0
        self.rx_errors: int = 0
        self.crc_errors: int = 0
        self.rx_overflow_count: int = 0
        self.last_rx_time: float = 0.0
        self.last_any_frame_at: float = 0.0
        self.last_heartbeat_ack_at: float = 0.0
        self.last_command_ack_at: float = 0.0
        self._last_any_frame_mono: float = 0.0
        self._last_heartbeat_ack_mono: float = 0.0
        self._last_command_ack_mono: float = 0.0

        # Pending ACKs
        self._pending_acks: Dict[int, tuple[asyncio.Future, int]] = {}
        self._ack_timeout: float = 1.0
        self.ack_timeouts: int = 0
        self.nack_count: int = 0
        self.last_command_sequence_attempted: Optional[int] = None
        self._last_crc_warning_mono: float = 0.0
        self.last_error: str = ""
        self.last_connect_error: str = ""
        self.disconnect_count: int = 0
        self.reconnect_attempts: int = 0

    # -------- Connection --------

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def connection_state(self) -> ConnectionState:
        return self._connection_state

    def attach_simulator(self, sim_stm32):
        """Attach a SimulatedStm32 instance (simulation mode)."""
        self._sim_stm32 = sim_stm32

    async def open(self):
        """Open the connection and start background tasks."""
        if self._running:
            return
        self._running = True

        if self._sim_stm32:
            logger.info("SIMULATION MODE — using virtual STM32")
            await self._sim_stm32.start()
            self._connected = True
            self._connection_state = ConnectionState.CONNECTED
            self.connection_generation += 1
            self._reset_connection_context()
        else:
            await self._connect_serial()
            if not self._connected:
                self._ensure_reconnect_task()

        if self._reader_task_handle is None or self._reader_task_handle.done():
            self._reader_task_handle = asyncio.create_task(
                self._reader_task(), name="serial-reader")
        if self._heartbeat_task_handle is None or self._heartbeat_task_handle.done():
            self._heartbeat_task_handle = asyncio.create_task(
                self._heartbeat_task(), name="serial-heartbeat")

    async def close(self):
        """Close the connection gracefully."""
        if not self._running and self._connection_state == ConnectionState.DISCONNECTED:
            return
        self._running = False

        # Best-effort safety commands. The application-level shutdown path may
        # already have sent these with ACK/confirmation.
        try:
            if self._connected:
                await self.send_frame(MsgType.SET_ALL_NEUTRAL)
                await self.send_frame(MsgType.DISARM)
        except Exception as exc:
            logger.warning("Safety commands during transport shutdown failed: %s", exc)

        tasks = (
            self._heartbeat_task_handle,
            self._reader_task_handle,
            self._reconnect_task_handle,
            self._serial_reader_task_handle,
        )
        for task in tasks:
            if task and task is not asyncio.current_task() and not task.done():
                task.cancel()
        for task in tasks:
            if task and task is not asyncio.current_task():
                try:
                    await task
                except asyncio.CancelledError:
                    pass
                except Exception as e:
                    logger.debug("Background task shutdown error: %s", e)

        for task in list(self._callback_tasks):
            task.cancel()
        if self._callback_tasks:
            await asyncio.gather(*self._callback_tasks, return_exceptions=True)
        self._callback_tasks.clear()

        if self._sim_stm32:
            await self._sim_stm32.stop()
        else:
            await self._close_serial_io()

        self._connected = False
        self._connection_state = ConnectionState.DISCONNECTED
        self._reset_connection_context()
        self._fail_pending_acks("transport closed")
        self._reader_task_handle = None
        self._heartbeat_task_handle = None
        self._reconnect_task_handle = None
        self._serial_reader_task_handle = None

    async def _connect_serial(self) -> bool:
        """Connect to a real serial port."""
        async with self._connection_lock:
            if not self._running:
                return False
            if self._connected:
                return True
            self._connection_state = ConnectionState.CONNECTING
            await self._close_serial_io()
            reader = None
            writer = None
            serial_obj = None
            try:
                try:
                    import serial_asyncio
                    reader, writer = await serial_asyncio.open_serial_connection(
                        url=self._port, baudrate=self._baudrate,
                    )
                except ImportError:
                    logger.warning("serial_asyncio not available, using basic serial")
                    import serial
                    serial_obj = serial.Serial(
                        self._port, self._baudrate, timeout=0)
                    reader = asyncio.StreamReader()
                    writer = _SerialWriter(serial_obj)

                self._reader = reader
                self._writer = writer
                opened_serial = serial_obj or getattr(
                    getattr(writer, "transport", None), "serial", None)
                if opened_serial is not None:
                    try:
                        opened_serial.reset_input_buffer()
                    except (AttributeError, OSError) as flush_error:
                        logger.debug(
                            "Serial input buffer reset unavailable: %s",
                            flush_error)
                if serial_obj is not None:
                    self._serial_reader_task_handle = asyncio.create_task(
                        self._serial_reader_loop(serial_obj),
                        name="serial-pyserial-reader")

                logger.info("Connected to %s at %d baud", self._port, self._baudrate)
                self._connected = True
                self._connection_state = ConnectionState.CONNECTED
                self.connection_generation += 1
                self.last_connect_error = ""
                self.last_error = ""
                self._reset_connection_context()
                return True
            except Exception as e:
                self._connected = False
                self._connection_state = ConnectionState.DISCONNECTED
                self.last_connect_error = str(e)
                logger.error("Failed to open %s: %s", self._port, e)
                if writer is not None:
                    try:
                        writer.close()
                    except Exception as close_error:
                        logger.debug("Failed to close unsuccessful serial writer: %s", close_error)
                return False

    def _reset_connection_context(self):
        self._rx_buf.clear()
        self.last_rx_time = 0.0
        self.last_any_frame_at = 0.0
        self.last_heartbeat_ack_at = 0.0
        self.last_command_ack_at = 0.0
        self._last_any_frame_mono = 0.0
        self._last_heartbeat_ack_mono = 0.0
        self._last_command_ack_mono = 0.0

    async def _close_serial_io(self):
        current = asyncio.current_task()
        serial_reader_task = self._serial_reader_task_handle
        self._serial_reader_task_handle = None
        if (serial_reader_task and serial_reader_task is not current
                and not serial_reader_task.done()):
            serial_reader_task.cancel()
            await asyncio.gather(serial_reader_task, return_exceptions=True)

        writer = self._writer
        self._reader = None
        self._writer = None
        if writer is None:
            return
        try:
            writer.close()
        except Exception as exc:
            logger.debug("Serial writer close failed: %s", exc)
            return
        wait_closed = getattr(writer, "wait_closed", None)
        if wait_closed is not None:
            try:
                await asyncio.wait_for(wait_closed(), timeout=1.0)
            except (AttributeError, NotImplementedError, asyncio.TimeoutError) as exc:
                logger.debug("Serial writer wait_closed unavailable: %s", exc)
            except Exception as exc:
                logger.debug("Serial writer wait_closed failed: %s", exc)
    async def _serial_reader_loop(self, ser):
        """Fallback reader loop using pyserial directly."""
        while self._running and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    if self._reader is not None:
                        self._reader.feed_data(data)
                await asyncio.sleep(0.001)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                await self._handle_disconnect(
                    f"fallback serial reader failed: {exc}")
                break

    async def _reconnect_loop(self):
        """Auto-reconnect with exponential backoff."""
        retry = 0
        while self._running and not self._connected:
            if (self._max_reconnect_retries > 0
                    and retry >= self._max_reconnect_retries):
                logger.error(
                    "Serial reconnect stopped after %d failed attempts",
                    self._max_reconnect_retries)
                break
            delay = self._reconnect_delays[min(retry, len(self._reconnect_delays) - 1)]
            logger.info("Reconnecting in %ss...", delay)
            await asyncio.sleep(delay)
            if not self._running or self._connected:
                break
            self.reconnect_attempts += 1
            if await self._connect_serial():
                break
            retry += 1

    def _ensure_reconnect_task(self):
        if self._sim_stm32 or not self._running:
            return
        if self._reconnect_task_handle and not self._reconnect_task_handle.done():
            return
        self._reconnect_task_handle = asyncio.create_task(
            self._reconnect_loop(), name="serial-reconnect")

    async def _handle_disconnect(self, reason: str):
        async with self._disconnect_lock:
            was_connected = self._connected
            if was_connected:
                logger.warning("Serial disconnected: %s", reason)
                self.disconnect_count += 1
            self._connection_state = ConnectionState.DISCONNECTING
            self._connected = False
            self.last_error = reason
            self._fail_pending_acks(reason)
            await self._cancel_callback_tasks()
            self._reset_connection_context()
            if not self._sim_stm32:
                await self._close_serial_io()
            self._connection_state = ConnectionState.DISCONNECTED
        self._ensure_reconnect_task()

    def _fail_pending_acks(self, reason: str):
        for future, _msg_type in list(self._pending_acks.values()):
            if not future.done():
                future.set_exception(ConnectionError(reason))
        self._pending_acks.clear()

    async def _cancel_callback_tasks(self):
        current = asyncio.current_task()
        tasks = [
            task for task in self._callback_tasks
            if task is not current and not task.done()
        ]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    # -------- Frame I/O --------

    def on_frame(self, callback: Callable[[int, int, bytes], Awaitable[None]]):
        """Register a callback for received frames: async fn(msg_type, seq, payload)."""
        self._frame_callbacks.append(callback)

    def remove_frame_callback(self, callback: Callable[[int, int, bytes], Awaitable[None]]):
        """Remove a previously registered frame callback."""
        try:
            self._frame_callbacks.remove(callback)
        except ValueError:
            pass

    async def send_frame(self, msg_type: int, payload: bytes = b"",
                         expect_ack: bool = False) -> Optional[int]:
        """Send a protocol frame. Returns the sequence number if expect_ack is set."""
        if not self._connected:
            self.last_error = "transport not connected"
            return None

        async with self._write_lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFF

            frame = encode_frame(msg_type, seq, payload)

            if expect_ack:
                loop = asyncio.get_event_loop()
                future = loop.create_future()
                self._pending_acks[seq] = (future, int(msg_type))
                self.last_command_sequence_attempted = seq
            else:
                future = None

            try:
                await self._write(frame)
            except Exception as e:
                if expect_ack:
                    self._pending_acks.pop(seq, None)
                await self._handle_disconnect(str(e))
                return None
            self.tx_frames += 1

        if expect_ack:
            try:
                ack_seq = await asyncio.wait_for(future, self._ack_timeout)
                self.last_error = ""
                return ack_seq
            except asyncio.TimeoutError:
                self._pending_acks.pop(seq, None)
                self.ack_timeouts += 1
                try:
                    msg_name = MsgType(msg_type).name
                except ValueError:
                    msg_name = str(msg_type)
                self.last_error = f"ACK timeout for {msg_name} seq={seq}"
                logger.warning("%s", self.last_error)
                return None
            except asyncio.CancelledError:
                self._pending_acks.pop(seq, None)
                if future is not None and not future.done():
                    future.cancel()
                raise
            except Exception as e:
                self._pending_acks.pop(seq, None)
                self.last_error = str(e)
                return None

        return None

    async def _write(self, data: bytes):
        """Write raw bytes to the transport."""
        if self._sim_stm32:
            await self._sim_stm32.feed_bytes(data)
        elif self._writer:
            self._writer.write(data)
            try:
                await self._writer.drain()
            except Exception as e:
                raise ConnectionError(f"serial write failed: {e}") from e
        else:
            raise ConnectionError("serial writer is unavailable")

    async def _reader_task(self):
        """Continuously read bytes from the transport and parse frames."""
        while self._running:
            try:
                if self._sim_stm32:
                    if not self._sim_stm32.connected:
                        if self._connected:
                            await self._handle_disconnect(
                                "simulated serial link disconnected")
                        await asyncio.sleep(0.05)
                        continue
                    if not self._connected:
                        self._connected = True
                        self._connection_state = ConnectionState.CONNECTED
                        self.connection_generation += 1
                        self.last_error = ""
                        self.last_connect_error = ""
                        self._reset_connection_context()
                    frame = await self._sim_stm32.read_frame_wait(0.05)
                    if frame:
                        self._process_raw_frame(frame)
                elif self._reader:
                    data = await self._reader.read(256)
                    if not data:
                        # Connection closed
                        await self._handle_disconnect("reader returned EOF")
                        continue
                    self._rx_buf.extend(data)
                    max_buffer = PROTO_BUF_SIZE * 4
                    if len(self._rx_buf) > max_buffer:
                        self.rx_overflow_count += 1
                        self.rx_errors += 1
                        self._rx_buf = self._rx_buf[-max_buffer:]
                    await self._parse_buffer()
                else:
                    await asyncio.sleep(0.1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("Reader error: %s", e)
                await self._handle_disconnect(str(e))
                await asyncio.sleep(0.05)

    async def _parse_buffer(self):
        """Parse complete frames from the RX buffer."""
        while True:
            idx = find_frame_start(bytes(self._rx_buf))
            if idx < 0:
                # Keep only the last byte (might be start of magic)
                self._rx_buf = (self._rx_buf[-1:]
                                if self._rx_buf[-1:] == bytes([MAGIC_0])
                                else bytearray())
                break

            if idx > 0:
                self._rx_buf = self._rx_buf[idx:]

            if len(self._rx_buf) < FRAME_OVERHEAD:
                break

            try:
                payload_len = struct.unpack_from("<H", self._rx_buf, 6)[0]
            except struct.error:
                break

            if payload_len > MAX_PAYLOAD:
                self.rx_errors += 1
                self._rx_buf = self._rx_buf[2:]
                continue

            frame_len = FRAME_OVERHEAD + payload_len + CRC_SIZE
            if len(self._rx_buf) < frame_len:
                break

            frame = bytes(self._rx_buf[:frame_len])
            result = decode_frame(frame)
            if result is None:
                self._record_invalid_frame(frame)
                self._rx_buf = self._rx_buf[2:]  # skip magic, try again
                continue

            self._rx_buf = self._rx_buf[frame_len:]

            self._process_raw_frame(frame)

    def _process_raw_frame(self, frame: bytes):
        """Process a complete, validated frame."""
        result = decode_frame(frame)
        if result is None:
            self._record_invalid_frame(frame)
            return

        msg_type, seq, payload = result
        self.rx_frames += 1
        now_wall = time.time()
        now_mono = time.monotonic()
        self.last_rx_time = now_wall
        self.last_any_frame_at = now_wall
        self._last_any_frame_mono = now_mono

        # Handle ACK/NACK internally
        if msg_type == MsgType.ACK and len(payload) == 2:
            ack = ProtoAck.unpack(payload)
            pending = self._pending_acks.pop(ack.ack_seq, None)
            future = pending[0] if pending else None
            if future and not future.done():
                self.last_command_ack_at = now_wall
                self._last_command_ack_mono = now_mono
                future.set_result(ack.ack_seq)
            elif pending is None:
                logger.debug("Ignoring ACK for unknown sequence %d", ack.ack_seq)

        elif msg_type == MsgType.NACK and len(payload) == 4:
            nack = ProtoNack.unpack(payload)
            self.nack_count += 1
            message = (f"NACK: seq={nack.rejected_sequence} "
                       f"type={nack.original_type} reason={nack.reason}")
            pending = self._pending_acks.get(nack.rejected_sequence)
            if pending is None:
                logger.warning("Ignoring NACK for unknown sequence %d",
                               nack.rejected_sequence)
            elif pending[1] != nack.original_type:
                logger.warning(
                    "Ignoring NACK type mismatch for sequence %d: pending=%d nack=%d",
                    nack.rejected_sequence, pending[1], nack.original_type)
            else:
                self._pending_acks.pop(nack.rejected_sequence, None)
                future = pending[0]
                self.last_error = message
                logger.warning("%s", message)
                if not future.done():
                    future.set_exception(RuntimeError(message))

        elif msg_type == MsgType.HEARTBEAT_ACK:
            self.last_heartbeat_ack_at = now_wall
            self._last_heartbeat_ack_mono = now_mono

        elif msg_type in (MsgType.ACK, MsgType.NACK):
            self.rx_errors += 1
            logger.warning("Ignoring malformed %s payload length %d",
                           MsgType(msg_type).name, len(payload))

        # Dispatch to registered callbacks
        for cb in self._frame_callbacks:
            try:
                task = asyncio.create_task(
                    self._run_callback(cb, msg_type, seq, payload))
                self._callback_tasks.add(task)
                task.add_done_callback(self._callback_tasks.discard)
            except Exception:
                logger.exception("Failed to schedule frame callback")

    def _record_invalid_frame(self, frame: bytes):
        self.rx_errors += 1
        crc_mismatch = False
        if len(frame) >= CRC_SIZE:
            received_crc = struct.unpack_from("<H", frame, len(frame) - CRC_SIZE)[0]
            crc_mismatch = received_crc != crc16(frame[:-CRC_SIZE])
        if crc_mismatch:
            self.crc_errors += 1
        now = time.monotonic()
        if now - self._last_crc_warning_mono >= 10.0:
            logger.warning(
                "Invalid frame (crc_mismatch=%s crc_errors=%d rx_errors=%d)",
                crc_mismatch, self.crc_errors, self.rx_errors)
            self._last_crc_warning_mono = now

    async def _run_callback(self, callback, msg_type: int,
                            seq: int, payload: bytes):
        try:
            result = callback(msg_type, seq, payload)
            if asyncio.iscoroutine(result):
                await result
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("Frame callback failed for type=0x%02X", msg_type)

    async def _heartbeat_task(self):
        """Periodically send HEARTBEAT frames."""
        while self._running:
            try:
                if self._connected:
                    hb = Heartbeat(heartbeat_timeout_ms=self._heartbeat_timeout_ms)
                    await self.send_frame(MsgType.HEARTBEAT, hb.pack())
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("Heartbeat error: %s", e)
            await asyncio.sleep(self._heartbeat_interval)


class _SerialWriter:
    """Minimal StreamWriter wrapper around pyserial.Serial."""
    def __init__(self, ser):
        self._ser = ser

    def write(self, data: bytes):
        self._ser.write(data)

    async def drain(self):
        self._ser.flush()

    def close(self):
        self._ser.close()

    async def wait_closed(self):
        pass
