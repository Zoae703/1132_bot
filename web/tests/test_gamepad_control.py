import asyncio
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import httpx
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "protocol" / "shared"))

from opi_console.config import AppConfig
from opi_console.serial_transport import SerialTransport
from opi_console.simulated_stm32 import SimulatedStm32
from opi_console.stm32_proxy import Stm32Proxy
from protocol import SafetyState
from web_backend.app import create_app
from web_backend.control_arbiter import ControlMode
from web_backend.gamepad_control import command_dict
from web_backend.gamepad_mapping import map_gamepad_state


ZERO_AXES = [0.0] * 6
ZERO_BUTTONS = [0] * 4
SESSION_ID = "gamepad_test_session"


def run(coro):
    return asyncio.run(coro)


class FakeGamepadWebSocket:
    def __init__(self):
        self.accepted = False
        self.closed = False
        self.close_code = None
        self.messages = []

    async def accept(self):
        self.accepted = True

    async def send_json(self, value):
        self.messages.append(value)

    async def close(self, code=1000):
        self.closed = True
        self.close_code = code


async def make_stack():
    config = AppConfig()
    transport = SerialTransport(
        heartbeat_interval=0.02,
        heartbeat_timeout_ms=200,
        config=config.transport_dict(),
    )
    transport._ack_timeout = 0.1
    sim = SimulatedStm32(
        heartbeat_timeout_ms=200,
        pwm_neutral=config.pwm.neutral_us,
        pwm_test_min=config.pwm.min_test_us,
        pwm_test_max=config.pwm.max_test_us,
        min_test_duration_ms=config.pwm.min_test_duration_ms,
        max_test_duration_ms=config.pwm.max_test_duration_ms,
        status_report_hz=config.telemetry.status_hz,
        sensor_report_hz=0.0,
    )
    transport.attach_simulator(sim)
    proxy = Stm32Proxy(transport, config=config)
    await transport.open()
    await asyncio.sleep(0.05)
    await proxy.request_status()
    app = create_app(proxy, transport, config=config)
    client = httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app),
        base_url="http://test",
    )
    return client, app, transport, proxy, sim


async def close_stack(client, transport):
    await client.aclose()
    await transport.close()


def packet(
    config,
    sequence,
    *,
    axes=None,
    buttons=None,
    control_enabled=True,
    gamepad_connected=True,
    mapped_override=None,
):
    axes = list(ZERO_AXES if axes is None else axes)
    buttons = list(ZERO_BUTTONS if buttons is None else buttons)
    mapped = map_gamepad_state(axes, buttons, config.gamepad)
    return json.dumps({
        "type": "gamepad_state",
        "version": 1,
        "session_id": SESSION_ID,
        "sequence": sequence,
        "client_time_ns": sequence * 1_000_000,
        "control_enabled": control_enabled,
        "gamepad_connected": gamepad_connected,
        "device": {"name": "synthetic-test-gamepad"},
        "axes": axes,
        "buttons": buttons,
        "hats": [[0, 0]],
        "mapped_command": (
            mapped_override
            if mapped_override is not None
            else command_dict(mapped.command)
        ),
    }, allow_nan=False)


async def connect_and_send_neutral(app):
    ws = FakeGamepadWebSocket()
    service = app.state.gamepad_service
    assert await service.connect(ws) is True
    await service.handle_message(
        ws, packet(app.state.config, 1))
    return service, ws


async def enable_gamepad(client, app):
    service, ws = await connect_and_send_neutral(app)
    arm = await client.post("/api/arm")
    assert arm.status_code == 200
    enabled = await client.post("/api/gamepad/enable")
    assert enabled.status_code == 200, enabled.text
    assert app.state.control_state.arbiter.mode == ControlMode.GAMEPAD
    return service, ws


@pytest.mark.parametrize(
    ("axes", "field", "sign"),
    [
        ([0, -1, 0, 0, 0, 0], "surge", 1),
        ([0, 1, 0, 0, 0, 0], "surge", -1),
        ([1, 0, 0, 0, 0, 0], "sway", 1),
        ([-1, 0, 0, 0, 0, 0], "sway", -1),
        ([0, 0, 0, 0, 1, 0], "yaw", 1),
        ([0, 0, 0, 0, -1, 0], "yaw", -1),
    ],
)
def test_gamepad_mapping_obeys_frd_axis_semantics(axes, field, sign):
    command = map_gamepad_state(
        axes, ZERO_BUTTONS, AppConfig().gamepad).command
    value = getattr(command, field)
    assert math.copysign(1, value) == sign


def test_y_is_up_a_is_down_and_a_y_conflict_is_zero():
    config = AppConfig().gamepad
    up = map_gamepad_state(ZERO_AXES, [0, 0, 0, 1], config)
    down = map_gamepad_state(ZERO_AXES, [1, 0, 0, 0], config)
    conflict = map_gamepad_state(ZERO_AXES, [1, 0, 0, 1], config)
    assert up.command.heave < 0
    assert down.command.heave > 0
    assert conflict.command.heave == 0
    assert conflict.heave_conflict is True


def test_axis3_b_and_x_do_not_affect_any_motion_axis():
    result = map_gamepad_state(
        [0, 0, 0, 1, 0, 0],
        [0, 1, 1, 0],
        AppConfig().gamepad,
    )
    assert set(result.command.values()) == {0.0}


def test_center_drift_is_zero_and_deadzone_edge_does_not_jump():
    config = AppConfig().gamepad
    center = map_gamepad_state(
        [0, -config.deadzone / 2, 0, 0, 0, 0],
        ZERO_BUTTONS,
        config,
    )
    edge = map_gamepad_state(
        [0, -(config.deadzone + 1e-6), 0, 0, 0, 0],
        ZERO_BUTTONS,
        config,
    )
    assert center.command.surge == 0
    assert 0 < edge.command.surge < 1e-5


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_non_finite_gamepad_axes_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        map_gamepad_state(
            [value, 0, 0, 0, 0, 0],
            ZERO_BUTTONS,
            AppConfig().gamepad,
        )


def test_unarmed_gamepad_cannot_enter_or_produce_nonzero_command():
    async def scenario():
        client, app, transport, _proxy, sim = await make_stack()
        try:
            service, ws = await connect_and_send_neutral(app)
            response = await client.post("/api/gamepad/enable")
            assert response.status_code == 409
            assert "ARM" in response.json()["detail"]

            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    2,
                    axes=[0, -1, 0, 0, 0, 0],
                ),
            )
            await service.process_once()
            assert set(sim.body_command.values()) == {0.0}
            assert ws.messages[-1]["reason"] == "mode_not_gamepad"
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_motor_test_and_gamepad_are_mutually_exclusive():
    async def scenario():
        client, app, transport, _proxy, _sim = await make_stack()
        try:
            _service, _ws = await connect_and_send_neutral(app)
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/enter-manual")).status_code == 200
            assert app.state.control_state.arbiter.mode == ControlMode.MOTOR_TEST
            rejected = await client.post("/api/gamepad/enable")
            assert rejected.status_code == 409
            assert "MOTOR_TEST" in rejected.json()["detail"]
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_gamepad_mode_is_published_only_after_stm32_activation():
    async def scenario():
        client, app, transport, proxy, _sim = await make_stack()
        try:
            service, _ws = await connect_and_send_neutral(app)
            assert (await client.post("/api/arm")).status_code == 200

            activation_started = asyncio.Event()
            allow_activation = asyncio.Event()
            real_enable_body_control = proxy.enable_body_control

            async def delayed_enable_body_control():
                activation_started.set()
                await allow_activation.wait()
                return await real_enable_body_control()

            proxy.enable_body_control = delayed_enable_body_control
            enable_task = asyncio.create_task(
                client.post("/api/gamepad/enable"))
            await asyncio.wait_for(activation_started.wait(), timeout=1)

            assert app.state.control_state.arbiter.mode == ControlMode.IDLE
            await service.process_once()
            assert app.state.control_state.safety_transition_active is False
            assert service.status_snapshot()["last_disconnect_reason"] is None

            allow_activation.set()
            response = await asyncio.wait_for(enable_task, timeout=1)
            assert response.status_code == 200, response.text
            assert app.state.control_state.arbiter.mode == ControlMode.GAMEPAD

            state = proxy.refresh_link_state()
            assert state.safety_state == SafetyState.ARMED_ACTIVE
            assert state.to_dict()["body_control_enabled"] is True
            assert (
                app.state.control_state.arbiter.snapshot()[
                    "last_release_reason"
                ]
                is None
            )
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_web_motion_and_gamepad_cannot_inject_commands_into_each_other():
    async def scenario():
        client, app, transport, _proxy, _sim = await make_stack()
        try:
            service, ws = await connect_and_send_neutral(app)
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/motion/enable")).status_code == 200
            assert app.state.control_state.arbiter.mode == (
                ControlMode.WEB_MOTION)
            rejected_gamepad = await client.post("/api/gamepad/enable")
            assert rejected_gamepad.status_code == 409
            assert "WEB_MOTION" in rejected_gamepad.json()["detail"]

            assert (await client.post("/api/motion/disable")).status_code == 200
            assert (await client.post("/api/disarm")).status_code == 200
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/gamepad/enable")).status_code == 200

            rejected_web = await client.post(
                "/api/motion/command",
                json={
                    "surge": 1,
                    "sway": 0,
                    "heave": 0,
                    "roll": 0,
                    "pitch": 0,
                    "yaw": 0,
                },
            )
            assert rejected_web.status_code == 409
            assert "WEB_MOTION required" in rejected_web.json()["detail"]

            await service.disconnect(ws, "test_cleanup")
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_300ms_timeout_zeros_six_axes_and_requires_center_before_resume():
    async def scenario():
        client, app, transport, _proxy, sim = await make_stack()
        try:
            service, ws = await enable_gamepad(client, app)
            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    2,
                    axes=[-1, 0, 0, 0, 0.5, 0],
                    buttons=[1, 0, 0, 0],
                ),
            )
            await service.process_once()
            assert any(value != 0 for value in sim.body_command.values())

            frame = service._latest
            service._latest = replace(
                frame,
                received_mono=(
                    frame.received_mono
                    - app.state.config.gamepad.zero_timeout_ms / 1000
                    - 0.001
                ),
            )
            await service.process_once()
            assert set(sim.body_command.values()) == {0.0}
            assert app.state.control_state.arbiter.mode == ControlMode.GAMEPAD
            assert service.status_snapshot()["resume_requires_neutral"] is True

            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    3,
                    axes=[-1, 0, 0, 0, 0, 0],
                ),
            )
            await service.process_once()
            assert set(sim.body_command.values()) == {0.0}
            assert ws.messages[-1]["reason"] == "center_controls_after_timeout"

            await service.handle_message(
                ws, packet(app.state.config, 4))
            await service.process_once()
            assert service.status_snapshot()["resume_requires_neutral"] is False
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_1000ms_timeout_exits_gamepad_and_disarms():
    async def scenario():
        client, app, transport, proxy, _sim = await make_stack()
        try:
            service, _ws = await enable_gamepad(client, app)
            frame = service._latest
            service._latest = replace(
                frame,
                received_mono=(
                    frame.received_mono
                    - app.state.config.gamepad.disconnect_timeout_ms / 1000
                    - 0.001
                ),
            )
            await service.process_once()
            state = proxy.refresh_link_state()
            assert state.safety_state == SafetyState.DISARMED
            assert set(state.confirmed_pwm) == {1500}
            assert app.state.control_state.arbiter.mode == ControlMode.IDLE
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_websocket_disconnect_immediately_zeros_and_disarms():
    async def scenario():
        client, app, transport, proxy, sim = await make_stack()
        try:
            service, ws = await enable_gamepad(client, app)
            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    2,
                    axes=[0, -1, 0, 0, 0, 0],
                ),
            )
            await service.process_once()
            assert sim.body_command.surge > 0

            await service.disconnect(ws, "test_websocket_disconnect")
            assert proxy.refresh_link_state().safety_state == SafetyState.DISARMED
            assert set(sim.pwm) == {1500}
            assert app.state.control_state.arbiter.mode == ControlMode.IDLE
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_old_sequence_cannot_overwrite_latest_command():
    async def scenario():
        client, app, transport, _proxy, _sim = await make_stack()
        try:
            service, ws = await connect_and_send_neutral(app)
            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    3,
                    axes=[0, -1, 0, 0, 0, 0],
                ),
            )
            newest = service._latest
            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    2,
                    axes=[0, 1, 0, 0, 0, 0],
                ),
            )
            assert service._latest is newest
            assert service._latest.sequence == 3
            assert service._latest.command.surge > 0
            assert ws.messages[-1]["reason"] == "sequence_not_increasing"
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_estop_rejects_all_later_gamepad_motion():
    async def scenario():
        client, app, transport, proxy, sim = await make_stack()
        try:
            service, ws = await enable_gamepad(client, app)
            estop = await client.post("/api/emergency-stop")
            assert estop.status_code == 200
            assert proxy.refresh_link_state().safety_state == (
                SafetyState.EMERGENCY_STOP)
            assert app.state.control_state.arbiter.mode == ControlMode.IDLE

            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    2,
                    axes=[-1, 0, 0, 0, 1, 0],
                    buttons=[1, 0, 0, 0],
                ),
            )
            await service.process_once()
            assert set(sim.pwm) == {1500}
            assert ws.messages[-1]["accepted"] is False
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_only_one_gamepad_websocket_can_own_the_lease():
    async def scenario():
        client, app, transport, _proxy, _sim = await make_stack()
        try:
            first = FakeGamepadWebSocket()
            second = FakeGamepadWebSocket()
            service = app.state.gamepad_service
            assert await service.connect(first) is True
            assert await service.connect(second) is False
            assert second.closed is True
            assert second.close_code == 4409
            assert second.messages[-1]["reason"] == "lease_already_owned"
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_client_mapped_command_is_verified_not_trusted():
    async def scenario():
        client, app, transport, _proxy, _sim = await make_stack()
        try:
            service, ws = await connect_and_send_neutral(app)
            await service.handle_message(
                ws,
                packet(
                    app.state.config,
                    2,
                    mapped_override={
                        "surge": 1,
                        "sway": 0,
                        "heave": 0,
                        "roll": 0,
                        "pitch": 0,
                        "yaw": 0,
                    },
                ),
            )
            assert service._latest.sequence == 1
            assert ws.messages[-1]["reason"] == "mapped_command_mismatch"
        finally:
            await close_stack(client, transport)

    run(scenario())
