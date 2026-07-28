"""
WebSocket manager for real-time telemetry broadcast.

Manages connected clients and periodically pushes status updates.
"""

import asyncio
import json
import logging
import time
import uuid
from typing import Any, Set, Optional

from fastapi import WebSocket

from opi_console.serial_transport import SerialTransport
from opi_console.stm32_proxy import Stm32Proxy
from opi_console.config import AppConfig, coerce_config
from protocol import SafetyState
from web_backend.control_state import ControlState

logger = logging.getLogger("opi_console.ws")


class WebSocketManager:
    """Manages WebSocket client connections and telemetry broadcast."""

    def __init__(self, proxy: Stm32Proxy, transport: SerialTransport,
                 control_state: Optional[ControlState] = None,
                 config: Optional[AppConfig] = None,
                 gamepad_service: Optional[Any] = None):
        self._proxy = proxy
        self._transport = transport
        self._control_state = control_state or ControlState()
        self._config = coerce_config(config or proxy.config)
        self._gamepad_service = gamepad_service
        self._clients: Set[WebSocket] = set()
        self._clients_lock = asyncio.Lock()
        self._send_lock = asyncio.Lock()
        self._broadcast_task: Optional[asyncio.Task] = None
        self._last_disconnect_task: Optional[asyncio.Task] = None
        self._sequence: int = 0
        self.session_id = uuid.uuid4().hex
        self._stopping = False
        self.last_disconnect_safety_result: Optional[dict] = None

    # -------- Connection management --------

    @property
    def client_count(self) -> int:
        return len(self._clients)

    async def start(self):
        self._stopping = False
        if self._broadcast_task is None or self._broadcast_task.done():
            self._broadcast_task = asyncio.create_task(
                self.broadcast_loop(), name="ws-broadcast")

    async def stop(self):
        self._stopping = True
        if self._broadcast_task and not self._broadcast_task.done():
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass
        async with self._clients_lock:
            clients = list(self._clients)
            self._clients.clear()
        for ws in clients:
            try:
                await ws.close()
            except Exception as exc:
                logger.debug("WebSocket close during shutdown failed: %s", exc)
        task = self._last_disconnect_task
        if task and not task.done():
            await asyncio.gather(task, return_exceptions=True)

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self._clients_lock:
            self._clients.add(ws)
        logger.info("WebSocket client connected (%d total)", len(self._clients))

    async def disconnect(self, ws: WebSocket):
        trigger_safety = False
        async with self._clients_lock:
            if ws not in self._clients:
                return
            self._clients.remove(ws)
            remaining = len(self._clients)
            if (remaining == 0 and not self._stopping
                    and self._control_state.begin_safety_transition(
                        "last_client_disconnected")):
                trigger_safety = True
        logger.info("WebSocket client disconnected (%d remaining)", remaining)
        if trigger_safety:
            self._last_disconnect_task = asyncio.create_task(
                self.handle_last_control_client_disconnected(),
                name="last-client-disconnect-safety",
            )

    async def handle_last_control_client_disconnected(self):
        """Neutralize and disarm without blocking the WebSocket handler."""
        timeout = self._config.safety.disconnect_command_timeout_s
        result = {
            "neutral": False,
            "exit_manual": False,
            "disarm": False,
        }
        logger.warning("Last control client disconnected; entering safe state")
        try:
            async with self._control_state.lock:
                try:
                    result["neutral"] = await self._proxy.force_neutral(
                        reason="last_client_disconnected",
                        confirm=True,
                        timeout=timeout,
                    )
                except Exception:
                    logger.exception(
                        "Unexpected failure neutralizing after last client disconnect")

                state = self._proxy.refresh_link_state()
                if state.safety_state == SafetyState.MANUAL_TEST:
                    try:
                        result["exit_manual"] = await asyncio.wait_for(
                            self._proxy.exit_manual(), timeout=timeout)
                    except asyncio.TimeoutError:
                        logger.error(
                            "EXIT_MANUAL timed out after last client disconnect")
                    except Exception:
                        logger.exception(
                            "EXIT_MANUAL failed after last client disconnect")
                else:
                    result["exit_manual"] = True

                # DISARM is attempted even when neutral or EXIT_MANUAL failed.
                try:
                    result["disarm"] = await asyncio.wait_for(
                        self._proxy.disarm(), timeout=timeout)
                    if result["disarm"]:
                        self._control_state.clear_motion_inhibit()
                except asyncio.TimeoutError:
                    logger.error("DISARM timed out after last client disconnect")
                except Exception:
                    logger.exception("DISARM failed after last client disconnect")
        finally:
            self._control_state.arbiter.force_idle(
                "last_client_disconnected")
            self._control_state.finish_safety_transition()
            self.last_disconnect_safety_result = result
            logger.warning(
                "Last-client safety result neutral=%s exit_manual=%s disarm=%s",
                result["neutral"], result["exit_manual"], result["disarm"])

    # -------- Client message handler --------

    async def handle_client_message(self, ws: WebSocket, data: str):
        """Handle a JSON message from a WebSocket client."""
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            return

        cmd = msg.get("cmd", "")

        if cmd == "query_status":
            await self._send_one(ws, self._status_message())
        elif cmd == "query_sensors":
            state = self._proxy.refresh_link_state()
            await self._send_one(
                ws, self._message("sensors", state.sensors_to_dict()))
        else:
            logger.debug("Ignoring unknown WebSocket command: %s", cmd)

    # -------- Broadcast --------

    async def broadcast_loop(self):
        """Periodically broadcast telemetry to all connected clients."""
        interval = 1.0 / self._config.telemetry.status_hz
        while True:
            try:
                if self._clients:
                    await self._broadcast_status()
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception:
                logger.exception("WebSocket broadcast task recovered from error")
                await asyncio.sleep(1)

    async def _broadcast_status(self):
        """Send current status to all clients."""
        await self._send_all(self._status_message())

    def _status_message(self) -> dict:
        state = self._proxy.refresh_link_state()
        self._control_state.reconcile_depth_hold(
            state.safety_state,
            bool(state.flags & 0x02),
            link_ready=bool(
                self._transport.connected
                and state.stm32_online
                and not state.status_stale
            ),
        )
        status_data = state.to_dict()
        sensors_data = state.sensors_to_dict()
        sensors_last_update = sensors_data.pop("last_update", 0.0)
        payload = {
            "mode": "SIMULATION" if self._transport._sim_stm32 else "REAL HARDWARE",
            "serial_connected": self._transport.connected,
            "stm32_online": state.stm32_online,
            **status_data,
            **sensors_data,
            "sensors_last_update": sensors_last_update,
            "tx_frames": self._transport.tx_frames,
            "rx_frames": self._transport.rx_frames,
            "rx_errors": self._transport.rx_errors,
            "crc_errors": self._transport.crc_errors,
            "ack_timeouts": self._transport.ack_timeouts,
            "nack_count": self._transport.nack_count,
            "backend_motion_inhibited": self._control_state.motion_inhibited,
            "backend_motion_inhibit_reason": (
                self._control_state.motion_inhibit_reason),
            "control_mode": self._control_state.arbiter.mode.value,
            "control_arbiter": self._control_state.arbiter.snapshot(),
            "gamepad": (
                self._gamepad_service.status_snapshot()
                if self._gamepad_service is not None else None
            ),
        }
        return self._message("status", payload)

    def _message(self, msg_type: str, payload: dict) -> dict:
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        ts = time.time()
        return {
            "type": msg_type,
            "session_id": self.session_id,
            "sequence": self._sequence,
            "timestamp": ts,
            "payload": payload,
            # Backward-compatible for the current frontend during rollout.
            "data": {
                **payload,
                "session_id": self.session_id,
                "timestamp": ts,
                "sequence": self._sequence,
            },
        }

    async def _broadcast_sensors(self):
        state = self._proxy.refresh_link_state()
        msg = self._message("sensors", state.sensors_to_dict())
        await self._send_all(msg)

    async def _send_all(self, msg: dict):
        """Send a message to all connected clients."""
        async with self._clients_lock:
            clients = list(self._clients)
        await self._send_targets(msg, clients)

    async def _send_one(self, ws: WebSocket, msg: dict):
        async with self._clients_lock:
            if ws not in self._clients:
                return
        await self._send_targets(msg, [ws])

    async def _send_targets(self, msg: dict, clients: list[WebSocket]):
        if not clients:
            return
        data = json.dumps(msg, allow_nan=False)
        timeout = self._config.web.websocket_send_timeout_s

        async def send(ws: WebSocket):
            try:
                await asyncio.wait_for(ws.send_text(data), timeout=timeout)
                return None
            except asyncio.TimeoutError:
                logger.warning("WebSocket send timed out after %.2fs", timeout)
                return ws
            except Exception:
                return ws

        async with self._send_lock:
            results = await asyncio.gather(*(send(ws) for ws in clients))
        dead = {ws for ws in results if ws is not None}
        if dead:
            for ws in dead:
                await self.disconnect(ws)
            logger.debug("Removed %d dead WebSocket clients", len(dead))
