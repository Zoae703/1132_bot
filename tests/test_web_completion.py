import asyncio
import json
import logging.handlers
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import httpx
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "protocol" / "shared"))

from opi_console.config import AppConfig, load_app_config
from opi_console.logger import setup_logging
from opi_console.main import (
    ProcessLockError,
    acquire_process_lock,
    apply_overrides,
    enter_safe_state,
    parse_args,
)
from opi_console.serial_transport import SerialTransport
from opi_console.simulated_stm32 import SimulatedStm32
from opi_console.stm32_proxy import Stm32Proxy
from protocol import MsgType, SafetyState, encode_frame
from web_backend.app import create_app
from web_backend.control_state import ControlState
from web_backend.motor_mapping import load_mapping, save_mapping
from web_backend.ws_manager import WebSocketManager


def run(coro):
    return asyncio.run(coro)


async def wait_until(predicate, timeout=1.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(interval)
    raise AssertionError("condition was not met before timeout")


async def make_stack(
    *,
    config=None,
    open_transport=True,
    sim_kwargs=None,
    frontend_dist=None,
):
    config = config or AppConfig()
    sim_kwargs = sim_kwargs or {}
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
        **sim_kwargs,
    )
    transport.attach_simulator(sim)
    proxy = Stm32Proxy(transport, config=config)
    if open_transport:
        await transport.open()
        await asyncio.sleep(0.05)
        await proxy.request_status()
    app = create_app(
        proxy,
        transport,
        config=config,
        frontend_dist=frontend_dist,
    )
    client = httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app),
        base_url="http://test",
    )
    return client, app, transport, proxy, sim


async def close_stack(client, transport):
    await client.aclose()
    await transport.close()


class FakeWebSocket:
    def __init__(self, fail_send=False):
        self.accepted = False
        self.closed = False
        self.fail_send = fail_send
        self.messages = []

    async def accept(self):
        self.accepted = True

    async def close(self):
        self.closed = True

    async def send_text(self, data):
        if self.fail_send:
            raise RuntimeError("disconnected")
        self.messages.append(data)


class FakeSafetyProxy:
    def __init__(self, state=SafetyState.MANUAL_TEST):
        self.config = AppConfig()
        self.safety_state = state
        self.calls = []
        self.neutral_result = True
        self.exit_result = True
        self.disarm_result = True
        self.neutral_started = None
        self.neutral_release = None
        self.neutral_exception = None

    def refresh_link_state(self):
        return SimpleNamespace(safety_state=self.safety_state)

    async def force_neutral(self, reason, confirm=True, timeout=None):
        self.calls.append(("neutral", reason, confirm, timeout))
        if self.neutral_started is not None:
            self.neutral_started.set()
        if self.neutral_release is not None:
            await self.neutral_release.wait()
        if self.neutral_exception is not None:
            raise self.neutral_exception
        return self.neutral_result

    async def exit_manual(self):
        self.calls.append(("exit_manual",))
        if self.exit_result:
            self.safety_state = SafetyState.ARMED_IDLE
        return self.exit_result

    async def disarm(self):
        self.calls.append(("disarm",))
        if self.disarm_result:
            self.safety_state = SafetyState.DISARMED
        return self.disarm_result


def fake_transport():
    return SimpleNamespace(_sim_stm32=None)


def test_last_client_disconnect_neutralizes_exits_manual_and_disarms():
    async def scenario():
        proxy = FakeSafetyProxy()
        control = ControlState()
        manager = WebSocketManager(
            proxy, fake_transport(), control_state=control, config=proxy.config)
        ws = FakeWebSocket()
        await manager.connect(ws)

        await manager.disconnect(ws)
        assert control.safety_transition_active is True
        await manager._last_disconnect_task

        assert [call[0] for call in proxy.calls] == [
            "neutral", "exit_manual", "disarm"]
        assert manager.last_disconnect_safety_result == {
            "neutral": True,
            "exit_manual": True,
            "disarm": True,
        }
        assert proxy.safety_state == SafetyState.DISARMED
        assert control.safety_transition_active is False

    run(scenario())


def test_only_the_last_unique_client_disconnect_triggers_safety_once():
    async def scenario():
        proxy = FakeSafetyProxy(state=SafetyState.ARMED_IDLE)
        manager = WebSocketManager(proxy, fake_transport(), config=proxy.config)
        first = FakeWebSocket()
        second = FakeWebSocket()
        await manager.connect(first)
        await manager.connect(second)

        await manager.disconnect(first)
        assert manager.client_count == 1
        assert manager._last_disconnect_task is None

        await manager.disconnect(first)
        assert manager._last_disconnect_task is None

        await manager.disconnect(second)
        await manager._last_disconnect_task
        await manager.disconnect(second)

        assert [call[0] for call in proxy.calls] == ["neutral", "disarm"]
        assert manager.client_count == 0

    run(scenario())


def test_disconnect_safety_attempts_disarm_after_neutral_or_exit_failure():
    async def scenario():
        proxy = FakeSafetyProxy()
        proxy.neutral_exception = asyncio.TimeoutError()
        proxy.exit_result = False
        manager = WebSocketManager(proxy, fake_transport(), config=proxy.config)
        ws = FakeWebSocket()
        await manager.connect(ws)

        await manager.disconnect(ws)
        await manager._last_disconnect_task

        assert [call[0] for call in proxy.calls] == [
            "neutral", "exit_manual", "disarm"]
        assert manager.last_disconnect_safety_result == {
            "neutral": False,
            "exit_manual": False,
            "disarm": True,
        }
        assert proxy.safety_state == SafetyState.DISARMED

    run(scenario())


def test_reconnect_does_not_cancel_an_in_progress_disconnect_safety_action():
    async def scenario():
        proxy = FakeSafetyProxy()
        proxy.neutral_started = asyncio.Event()
        proxy.neutral_release = asyncio.Event()
        manager = WebSocketManager(proxy, fake_transport(), config=proxy.config)
        first = FakeWebSocket()
        replacement = FakeWebSocket()
        await manager.connect(first)

        await manager.disconnect(first)
        await proxy.neutral_started.wait()
        await manager.connect(replacement)
        proxy.neutral_release.set()
        await manager._last_disconnect_task

        assert manager.client_count == 1
        assert proxy.safety_state == SafetyState.DISARMED
        assert [call[0] for call in proxy.calls][-1] == "disarm"
        await manager.stop()

    run(scenario())


def test_real_simulator_stack_is_disarmed_and_confirmed_neutral_after_disconnect():
    async def scenario():
        client, _app, transport, proxy, sim = await make_stack()
        manager = WebSocketManager(proxy, transport, config=proxy.config)
        ws = FakeWebSocket()
        try:
            assert await proxy.arm()
            assert await proxy.enter_manual()
            assert await proxy.set_pwm(0, 1530, 2000)
            assert sim.pwm[0] == 1530
            await manager.connect(ws)

            await manager.disconnect(ws)
            await manager._last_disconnect_task

            state = proxy.refresh_link_state()
            assert state.safety_state == SafetyState.DISARMED
            assert state.confirmed_pwm == [1500] * 8
            assert state.requested_pwm == [1500] * 8
            assert sim.safety_state == SafetyState.DISARMED
            assert sim.pwm == [1500] * 8
        finally:
            await manager.stop()
            await close_stack(client, transport)

    run(scenario())


def test_switching_pwm_channels_neutralizes_and_reenters_manual_mode():
    async def scenario():
        client, _app, transport, proxy, sim = await make_stack()
        try:
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/enter-manual")).status_code == 200
            first = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1530, "duration_ms": 2000},
            )
            assert first.status_code == 200
            await asyncio.sleep(0.31)

            second = await client.post(
                "/api/pwm/test",
                json={"channel": 1, "pwm_us": 1520, "duration_ms": 1000},
            )
            assert second.status_code == 200
            assert sim.safety_state == SafetyState.MANUAL_TEST
            assert sim.pwm[0] == 1500
            assert sim.pwm[1] == 1520
            assert sum(value != 1500 for value in sim.pwm) == 1
            assert proxy.state.safety_state == SafetyState.MANUAL_TEST
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_pwm_confirmation_waits_for_delayed_control_task_application():
    async def scenario():
        client, _app, transport, _proxy, sim = await make_stack()
        try:
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/enter-manual")).status_code == 200
            original_handler = sim._h_set_pwm

            async def delayed_application(sequence, payload):
                await original_handler(sequence, payload)
                channel = sim.active_channel
                if channel < 0:
                    return
                requested = sim.pwm[channel]
                sim.pwm[channel] = 1500

                async def apply_after_control_period():
                    await asyncio.sleep(0.035)
                    if (sim.safety_state == SafetyState.MANUAL_TEST
                            and sim.active_channel == channel):
                        sim.pwm[channel] = requested

                task = asyncio.create_task(apply_after_control_period())
                sim._track_delayed_task(task)

            sim._h_set_pwm = delayed_application
            started = time.monotonic()
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1530, "duration_ms": 500},
            )
            elapsed = time.monotonic() - started

            assert response.status_code == 200
            assert elapsed >= 0.03
            assert sim.pwm[0] == 1530
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_pwm_status_confirmation_timeout_fails_and_forces_neutral():
    async def scenario():
        client, _app, transport, _proxy, sim = await make_stack()
        try:
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/enter-manual")).status_code == 200
            original_handler = sim._h_set_pwm

            async def never_apply(sequence, payload):
                await original_handler(sequence, payload)
                sim.pwm = [1500] * 8

            sim._h_set_pwm = never_apply
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1530, "duration_ms": 200},
            )

            assert response.status_code == 504
            assert "status did not confirm" in response.json()["detail"]
            assert sim.pwm == [1500] * 8
            assert sim.safety_state == SafetyState.ARMED_IDLE
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_api_rejects_motion_commands_during_disconnect_safety_transition():
    async def scenario():
        client, app, transport, _proxy, _sim = await make_stack()
        try:
            assert app.state.control_state.begin_safety_transition(
                "last_client_disconnected")
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 409
            assert "Safety transition" in response.json()["detail"]
        finally:
            app.state.control_state.finish_safety_transition()
            await close_stack(client, transport)

    run(scenario())


def test_capabilities_are_complete_and_come_from_validated_config():
    async def scenario():
        config = AppConfig.model_validate({
            "pwm": {
                "min_test_us": 1460,
                "max_test_us": 1540,
                "min_test_duration_ms": 250,
                "default_timeout_ms": 600,
            },
            "telemetry": {"status_hz": 4, "sensors_hz": 8},
            "features": {"motor_mapping": False},
        })
        client, _app, transport, _proxy, _sim = await make_stack(
            config=config, open_transport=False)
        try:
            response = await client.get("/api/capabilities")
            assert response.status_code == 200
            data = response.json()
            assert data["protocol_version"] == 2
            assert data["channel_count"] == 8
            assert data["pwm"]["neutral_us"] == 1500
            assert data["pwm"]["min_test_us"] == 1460
            assert data["pwm"]["min_test_duration_ms"] == 250
            assert data["pwm"]["default_timeout_ms"] == 600
            assert "channel_count" not in data["pwm"]
            assert data["features"]["motor_mapping"] is False
            assert data["sensor_poll_hz"] == 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_invalid_or_missing_config_fails_before_service_start(tmp_path):
    missing = tmp_path / "missing.yaml"
    with pytest.raises(ValueError, match="does not exist"):
        load_app_config(missing)

    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(
        "pwm:\n  neutral_us: 1490\n  max_test_duration_ms: 5000\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="Invalid configuration"):
        load_app_config(invalid)


def test_app_lifespan_starts_one_sensor_poll_task_and_stops_background_tasks():
    async def scenario():
        config = AppConfig.model_validate({
            "telemetry": {
                "status_hz": 5,
                "sensors_hz": 20,
                "request_timeout_s": 0.05,
            }
        })
        client, app, transport, proxy, _sim = await make_stack(config=config)
        try:
            async with app.router.lifespan_context(app):
                await wait_until(
                    lambda: proxy.state.last_sensor_report_at > 0,
                    timeout=1.0,
                )
                task = proxy._sensor_poll_task
                await proxy.start_background_tasks()
                assert proxy._sensor_poll_task is task
                assert proxy.sensor_poll_task_running is True
                assert app.state.ws_manager._broadcast_task is not None
                assert not app.state.ws_manager._broadcast_task.done()

            assert proxy.sensor_poll_task_running is False
            assert app.state.ws_manager._broadcast_task.done()
        finally:
            await close_stack(client, transport)

        assert transport._reader_task_handle is None
        assert transport._heartbeat_task_handle is None

    run(scenario())


def test_sensor_polling_does_not_queue_requests_while_offline():
    async def scenario():
        config = AppConfig.model_validate({
            "telemetry": {"sensors_hz": 20, "request_timeout_s": 0.05}
        })
        client, app, transport, proxy, _sim = await make_stack(
            config=config, open_transport=False)
        try:
            async with app.router.lifespan_context(app):
                await asyncio.sleep(0.15)
                assert proxy.sensor_poll_requests == 0
                assert proxy._sensor_request_in_flight is False
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_sensor_poll_task_recovers_after_unexpected_exception():
    async def scenario():
        config = AppConfig.model_validate({
            "telemetry": {"sensors_hz": 20, "request_timeout_s": 0.05}
        })
        client, app, transport, proxy, _sim = await make_stack(config=config)
        original_request = proxy.request_sensors
        attempts = 0

        async def flaky_request():
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise RuntimeError("injected sensor polling failure")
            return await original_request()

        proxy.request_sensors = flaky_request
        try:
            async with app.router.lifespan_context(app):
                await wait_until(lambda: attempts >= 2, timeout=1.0)
                await wait_until(
                    lambda: proxy.state.last_sensor_report_at > 0,
                    timeout=1.0,
                )
                assert proxy.sensor_poll_failures >= 1
                assert proxy.sensor_poll_task_running is True
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_sensor_polling_recovers_after_simulated_serial_reconnect():
    async def scenario():
        config = AppConfig.model_validate({
            "telemetry": {
                "sensors_hz": 20,
                "request_timeout_s": 0.05,
                "stm32_online_timeout_s": 0.3,
            }
        })
        client, app, transport, proxy, sim = await make_stack(config=config)
        try:
            async with app.router.lifespan_context(app):
                await wait_until(lambda: proxy.sensor_poll_requests >= 1)
                await sim.disconnect()
                await wait_until(lambda: not transport.connected)
                await asyncio.sleep(0.12)
                requests_while_offline = proxy.sensor_poll_requests
                await asyncio.sleep(0.12)
                assert proxy.sensor_poll_requests == requests_while_offline

                await sim.reconnect()
                await wait_until(lambda: transport.connected)
                await wait_until(
                    lambda: proxy.refresh_link_state().stm32_online,
                    timeout=1.0,
                )
                await wait_until(
                    lambda: proxy.sensor_poll_requests > requests_while_offline,
                    timeout=1.0,
                )
                await wait_until(
                    lambda: proxy.state.last_sensor_report_at > 0,
                    timeout=1.0,
                )
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_reconnect_keeps_old_status_stale_until_new_status_report():
    async def scenario():
        client, _app, transport, proxy, sim = await make_stack()
        try:
            assert proxy.state.status_stale is False
            previous_generation = transport.connection_generation
            await sim.disconnect()
            await wait_until(lambda: not transport.connected)
            await sim.reconnect()
            await wait_until(
                lambda: transport.connection_generation > previous_generation,
                timeout=1.0,
            )
            await wait_until(
                lambda: proxy.refresh_link_state().stm32_online,
                timeout=1.0,
            )
            assert proxy.state.status_stale is True
            assert proxy.state.last_status_report_at == 0.0

            assert await proxy.request_status() is not None
            assert proxy.state.status_stale is False
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_heartbeat_does_not_make_old_sensor_data_fresh():
    async def scenario():
        config = AppConfig.model_validate({
            "telemetry": {
                "sensors_stale_timeout_s": 0.05,
                "stm32_online_timeout_s": 0.3,
            }
        })
        client, _app, transport, proxy, _sim = await make_stack(config=config)
        try:
            assert await proxy.request_sensors() is not None
            sensor_timestamp = proxy.state.last_sensor_report_at
            await asyncio.sleep(0.08)
            state = proxy.refresh_link_state()
            assert state.stm32_online is True
            assert state.last_sensor_report_at == sensor_timestamp
            assert state.sensors_stale is True
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_non_finite_sensor_values_are_serialized_as_json_null():
    async def scenario():
        client, app, transport, proxy, _sim = await make_stack()
        try:
            proxy._state.depth_m = float("nan")
            proxy._state.pressure_mbar = float("inf")
            proxy._state.accel = [float("nan"), float("inf"), float("-inf")]

            response = await client.get("/api/sensors")
            assert response.status_code == 200
            assert response.json()["depth_m"] is None
            assert response.json()["pressure_mbar"] is None
            assert response.json()["accel"] == [None, None, None]

            message = app.state.ws_manager._status_message()
            encoded = json.dumps(message, allow_nan=False)
            assert json.loads(encoded)["payload"]["depth_m"] is None
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_health_and_api_routes_are_not_shadowed_and_spa_routes_fallback(tmp_path):
    async def scenario():
        dist = tmp_path / "dist"
        dist.mkdir()
        (dist / "index.html").write_text(
            "<!doctype html><title>ROV SPA marker</title>", encoding="utf-8")
        client, _app, transport, _proxy, _sim = await make_stack(
            open_transport=False, frontend_dist=dist)
        try:
            health = await client.get("/health")
            assert health.status_code == 200
            assert health.headers["content-type"].startswith("application/json")

            capabilities = await client.get("/api/capabilities")
            assert capabilities.status_code == 200
            assert capabilities.headers["content-type"].startswith(
                "application/json")

            missing_api = await client.get("/api/not-found")
            assert missing_api.status_code == 404
            assert missing_api.headers["content-type"].startswith(
                "application/json")

            spa = await client.get("/dashboard")
            assert spa.status_code == 200
            assert "ROV SPA marker" in spa.text
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_websocket_messages_share_a_session_and_have_monotonic_sequences():
    async def scenario():
        proxy = FakeSafetyProxy(state=SafetyState.DISARMED)
        manager = WebSocketManager(proxy, fake_transport(), config=proxy.config)
        first = FakeWebSocket()
        second = FakeWebSocket()
        await manager.connect(first)
        await manager.connect(second)

        one = manager._message("status", {"value": 1})
        two = manager._message("sensors", {"value": 2})
        await manager._send_all(one)
        await manager._send_all(two)

        assert one["session_id"] == two["session_id"] == manager.session_id
        assert two["sequence"] == one["sequence"] + 1
        assert one["timestamp"] == one["data"]["timestamp"]
        assert one["session_id"] == one["data"]["session_id"]
        for ws in (first, second):
            messages = [json.loads(item) for item in ws.messages]
            assert [item["sequence"] for item in messages] == [1, 2]
            assert {item["session_id"] for item in messages} == {
                manager.session_id}

        other = WebSocketManager(proxy, fake_transport(), config=proxy.config)
        assert other.session_id != manager.session_id
        await manager.stop()
        await other.stop()

    run(scenario())


@pytest.mark.parametrize(
    ("state", "active_channel", "control", "floating", "angle", "estop", "expected"),
    [
        (SafetyState.DISARMED, -1, False, False, False, False, 0x00),
        (SafetyState.ARMED_IDLE, -1, False, False, False, False, 0x00),
        (SafetyState.ARMED_ACTIVE, -1, True, True, True, False, 0x07),
        (SafetyState.MANUAL_TEST, -1, False, False, False, False, 0x00),
        (SafetyState.MANUAL_TEST, 0, False, False, False, False, 0x08),
        (SafetyState.COMM_LOST, -1, False, False, False, False, 0x00),
        (SafetyState.EMERGENCY_STOP, -1, False, False, False, True, 0x10),
    ],
)
def test_simulator_status_flags_match_firmware_state(
    state, active_channel, control, floating, angle, estop, expected
):
    sim = SimulatedStm32()
    sim.safety_state = state
    sim.active_channel = active_channel
    sim.control_enable = control
    sim.float_enabled = floating
    sim.angle_enabled = angle
    sim.estop_locked = estop
    assert sim._status_flags() == expected


def test_motor_mapping_api_validates_and_persists_atomically(tmp_path):
    async def scenario():
        mapping_file = tmp_path / "config" / "mapping.json"
        config = AppConfig.model_validate({
            "motor_mapping": {"file": str(mapping_file)}
        })
        client, _app, transport, _proxy, _sim = await make_stack(
            config=config, open_transport=False)
        valid = {
            "mappings": [{
                "channel": 0,
                "physical_name": "front-left",
                "direction": "horizontal",
                "reversed": False,
                "neutral_us": 1500,
                "safe_min_us": 1400,
                "safe_max_us": 1600,
                "notes": "bench verified",
            }]
        }
        try:
            response = await client.post("/api/motor-mapping", json=valid)
            assert response.status_code == 200
            assert mapping_file.exists()
            assert not list(mapping_file.parent.glob("*.tmp"))

            loaded = await client.get("/api/motor-mapping")
            assert loaded.status_code == 200
            assert len(loaded.json()["mappings"]) == 8
            assert loaded.json()["mappings"][0]["physical_name"] == "front-left"

            invalid_payloads = [
                {"mappings": [valid["mappings"][0], valid["mappings"][0]]},
                {"mappings": [{**valid["mappings"][0], "channel": 8}]},
                {"mappings": [{**valid["mappings"][0], "neutral_us": 1490}]},
                {"mappings": [{**valid["mappings"][0], "safe_min_us": 1510}]},
                {"mappings": [{**valid["mappings"][0], "safe_min_us": 1200}]},
            ]
            for payload in invalid_payloads:
                rejected = await client.post("/api/motor-mapping", json=payload)
                assert rejected.status_code == 422
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_corrupted_mapping_file_fails_closed_to_safe_defaults(tmp_path):
    mapping_file = tmp_path / "mapping.json"
    mapping_file.write_text("{not-json", encoding="utf-8")
    data = load_mapping(mapping_file)
    assert len(data["mappings"]) == 8
    assert all(item["neutral_us"] == 1500 for item in data["mappings"])

    mapping_file.write_text(
        json.dumps({
            "mappings": [{
                "channel": 0,
                "neutral_us": 1700,
                "safe_min_us": 1800,
                "safe_max_us": 1200,
            }]
        }),
        encoding="utf-8",
    )
    data = load_mapping(mapping_file)
    assert data["mappings"][0]["neutral_us"] == 1500
    assert data["mappings"][0]["safe_min_us"] == 1450
    assert data["mappings"][0]["safe_max_us"] == 1550


def test_atomic_mapping_save_preserves_previous_file_on_replace_failure(
    tmp_path, monkeypatch
):
    mapping_file = tmp_path / "mapping.json"
    original = [{"channel": 0, "neutral_us": 1500}]
    save_mapping(original, mapping_file)
    previous = mapping_file.read_text(encoding="utf-8")

    def fail_replace(_source, _destination):
        raise OSError("simulated replace failure")

    monkeypatch.setattr("web_backend.motor_mapping.os.replace", fail_replace)
    with pytest.raises(OSError, match="simulated replace failure"):
        save_mapping([{"channel": 1, "neutral_us": 1500}], mapping_file)

    assert mapping_file.read_text(encoding="utf-8") == previous
    assert not list(tmp_path.glob("*.tmp"))


def test_log_endpoint_uses_project_independent_absolute_path(tmp_path, monkeypatch):
    async def scenario():
        event_file = tmp_path / "events.log"
        event_file.write_text("first\nsecond\n", encoding="utf-8")
        config = AppConfig.model_validate({
            "logging": {"event_file": str(event_file)}
        })
        other_cwd = tmp_path / "other"
        other_cwd.mkdir()
        monkeypatch.chdir(other_cwd)
        client, _app, transport, _proxy, _sim = await make_stack(
            config=config, open_transport=False)
        try:
            response = await client.get("/api/logs?lines=1")
            assert response.status_code == 200
            assert response.json()["events"] == ["second\n"]
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_environment_and_cli_overrides_are_revalidated(monkeypatch):
    monkeypatch.setenv("ROV_SIMULATE", "true")
    monkeypatch.setenv("ROV_SERIAL_PORT", "/dev/ttyTEST")
    monkeypatch.setenv("ROV_SERIAL_BAUD", "115200")
    monkeypatch.setenv("ROV_WEB_HOST", "127.0.0.1")
    monkeypatch.setenv("ROV_WEB_PORT", "8123")
    monkeypatch.setenv("ROV_LOG_LEVEL", "DEBUG")

    args = parse_args([])
    effective = apply_overrides(AppConfig(), args)
    assert effective.simulation.enabled is True
    assert effective.serial.port == "/dev/ttyTEST"
    assert effective.serial.baudrate == 115200
    assert effective.web.host == "127.0.0.1"
    assert effective.web.port == 8123
    assert effective.logging.level == "DEBUG"

    args.baud = 57600
    with pytest.raises(ValueError, match="115200"):
        apply_overrides(AppConfig(), args)


def test_unknown_config_keys_and_invalid_log_levels_are_rejected():
    with pytest.raises(ValueError, match="extra"):
        AppConfig.model_validate({"pwm": {"neutrl_us": 1500}})
    with pytest.raises(ValueError, match="logging.level"):
        AppConfig.model_validate({"logging": {"level": "verbose"}})


def test_process_lock_rejects_a_second_backend_instance(tmp_path):
    lock_file = tmp_path / "console.lock"
    with acquire_process_lock(lock_file):
        with pytest.raises(ProcessLockError, match="Another 1132_bot"):
            with acquire_process_lock(lock_file):
                raise AssertionError("second process lock unexpectedly acquired")

    with acquire_process_lock(lock_file):
        assert "pid=" in lock_file.read_text(encoding="utf-8")


def test_logging_rotation_uses_validated_configuration(tmp_path):
    logger = setup_logging(
        log_file=str(tmp_path / "service.log"),
        event_file=str(tmp_path / "events.log"),
        console=False,
        max_bytes=12345,
        backup_count=2,
    )
    try:
        handlers = [
            handler for handler in logger.handlers
            if isinstance(handler, logging.handlers.RotatingFileHandler)
        ]
        assert len(handlers) == 2
        assert all(handler.maxBytes == 12345 for handler in handlers)
        assert all(handler.backupCount == 2 for handler in handlers)
    finally:
        for handler in list(logger.handlers):
            handler.close()
        logger.handlers.clear()


def test_process_boundary_safety_attempts_disarm_after_neutral_failure():
    class Proxy:
        def __init__(self):
            self.calls = []

        async def force_neutral(self, reason, confirm=True, timeout=None):
            self.calls.append(("neutral", reason, confirm, timeout))
            return False

        async def disarm(self):
            self.calls.append(("disarm",))
            return True

    async def scenario():
        proxy = Proxy()
        transport = SimpleNamespace(connected=True)
        result = await enter_safe_state(
            proxy, transport, timeout_s=0.1, reason="test_boundary")
        assert result == {"neutral": False, "disarm": True}
        assert [call[0] for call in proxy.calls] == ["neutral", "disarm"]

    run(scenario())


def test_disarm_does_not_clear_a_latched_emergency_stop():
    async def scenario():
        client, _app, transport, proxy, sim = await make_stack()
        try:
            assert await proxy.emergency_stop()
            assert sim.estop_locked is True
            response = await client.post("/api/disarm")
            assert response.status_code == 200
            assert "remains latched" in response.json()["message"]
            assert response.json()["safety_state"] == "EMERGENCY_STOP"
            assert sim.estop_locked is True
            assert sim.safety_state == SafetyState.EMERGENCY_STOP
            assert sim.pwm == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_slow_websocket_is_removed_without_blocking_a_live_client():
    class SlowWebSocket(FakeWebSocket):
        async def send_text(self, data):
            await asyncio.Event().wait()

    async def scenario():
        config = AppConfig.model_validate({
            "web": {"websocket_send_timeout_s": 0.02}
        })
        proxy = FakeSafetyProxy(state=SafetyState.DISARMED)
        manager = WebSocketManager(
            proxy, fake_transport(), config=config)
        slow = SlowWebSocket()
        live = FakeWebSocket()
        await manager.connect(slow)
        await manager.connect(live)
        started = time.monotonic()

        await manager._send_all(manager._message("status", {"ok": True}))

        assert time.monotonic() - started < 0.2
        assert len(live.messages) == 1
        assert manager.client_count == 1
        assert manager._last_disconnect_task is None
        await manager.stop()

    run(scenario())


def test_serial_parser_counts_crc_failure_and_resynchronizes_after_noise():
    async def scenario():
        transport = SerialTransport()
        received = []

        async def on_frame(msg_type, sequence, payload):
            received.append((msg_type, sequence, payload))

        transport.on_frame(on_frame)
        corrupted = bytearray(encode_frame(MsgType.NOP, 4, b"bad"))
        corrupted[-1] ^= 0xFF
        good = encode_frame(MsgType.LOG_MESSAGE, 5, b"good")
        transport._rx_buf.extend(b"noise" + bytes(corrupted) + good[:5])
        await transport._parse_buffer()
        transport._rx_buf.extend(good[5:])
        await transport._parse_buffer()
        await asyncio.sleep(0)

        assert transport.crc_errors >= 1
        assert transport.rx_errors >= transport.crc_errors
        assert received == [(MsgType.LOG_MESSAGE, 5, b"good")]

    run(scenario())


def test_cancelling_ack_wait_removes_pending_sequence():
    async def scenario():
        transport = SerialTransport()
        transport._connected = True

        async def discard_write(_data):
            return None

        transport._write = discard_write
        task = asyncio.create_task(
            transport.send_frame(MsgType.ARM, expect_ack=True))
        await wait_until(lambda: bool(transport._pending_acks))
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert transport._pending_acks == {}

    run(scenario())


def test_host_command_sequence_is_not_reused_on_reconnect():
    transport = SerialTransport()
    transport._seq = 0x4321
    transport._reset_connection_context()
    assert transport._seq == 0x4321

    transport._seq = 0xFFFF
    transport._reset_connection_context()
    assert transport._seq == 0xFFFF


def test_cancelled_pwm_http_request_forces_confirmed_neutral():
    async def scenario():
        client, app, transport, _proxy, sim = await make_stack()
        transport._ack_timeout = 0.3
        try:
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/enter-manual")).status_code == 200
            sim.ack_delay_ms = 80
            request = asyncio.create_task(client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1530, "duration_ms": 1000},
            ))
            await wait_until(lambda: sim.pwm[0] == 1530)
            request.cancel()
            with pytest.raises(asyncio.CancelledError):
                await request

            await wait_until(lambda: sim.pwm == [1500] * 8, timeout=1.0)
            assert app.state.control_state.safety_transition_active is False
            assert transport._pending_acks == {}
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_emergency_stop_preempts_an_in_flight_pwm_command():
    async def scenario():
        client, app, transport, proxy, sim = await make_stack()
        transport._ack_timeout = 0.3
        try:
            assert (await client.post("/api/arm")).status_code == 200
            assert (await client.post("/api/enter-manual")).status_code == 200
            sim.ack_delay_ms = 60
            pwm_request = asyncio.create_task(client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1530, "duration_ms": 1000},
            ))
            await wait_until(lambda: sim.pwm[0] == 1530)
            estop = await client.post("/api/emergency-stop")
            pwm_result = await pwm_request

            assert estop.status_code == 200
            assert pwm_result.status_code != 200
            assert sim.safety_state == SafetyState.EMERGENCY_STOP
            assert sim.estop_locked is True
            assert sim.pwm == [1500] * 8
            await proxy.request_status()
            assert proxy.state.confirmed_pwm == [1500] * 8
            assert app.state.control_state.estop_in_progress is False
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_duplicate_estop_request_is_rejected_while_first_is_in_progress():
    async def scenario():
        client, app, transport, _proxy, sim = await make_stack()
        transport._ack_timeout = 0.3
        sim.ack_delay_ms = 80
        try:
            first = asyncio.create_task(client.post("/api/emergency-stop"))
            await wait_until(lambda: app.state.control_state.estop_in_progress)
            second = await client.post("/api/emergency-stop")
            first_response = await first
            assert first_response.status_code == 200
            assert second.status_code == 409
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_unconfirmed_estop_latches_backend_motion_until_disarm_is_confirmed():
    async def scenario():
        client, app, transport, _proxy, sim = await make_stack()
        transport._ack_timeout = 0.04
        sim.drop_acks = True
        try:
            response = await client.post("/api/emergency-stop")
            assert response.status_code == 504
            assert sim.safety_state == SafetyState.EMERGENCY_STOP
            assert sim.pwm == [1500] * 8
            assert app.state.control_state.motion_inhibited is True

            status = (await client.get("/api/status")).json()
            assert status["backend_motion_inhibited"] is True
            assert status["backend_motion_inhibit_reason"] == (
                "emergency_stop_unconfirmed")

            arm = await client.post("/api/arm")
            assert arm.status_code == 409
            assert "Motion is inhibited" in arm.json()["detail"]

            sim.drop_acks = False
            disarm = await client.post("/api/disarm")
            assert disarm.status_code == 200
            assert app.state.control_state.motion_inhibited is False
            assert sim.estop_locked is True
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_disabled_manual_and_mapping_features_are_enforced_server_side(tmp_path):
    async def scenario():
        config = AppConfig.model_validate({
            "features": {"manual_pwm": False, "motor_mapping": False},
            "motor_mapping": {"file": str(tmp_path / "mapping.json")},
        })
        client, _app, transport, _proxy, _sim = await make_stack(config=config)
        try:
            assert (await client.post("/api/enter-manual")).status_code == 403
            pwm = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert pwm.status_code == 403
            assert (await client.get("/api/motor-mapping")).status_code == 403
            assert (await client.post(
                "/api/motor-mapping", json={"mappings": []})).status_code == 403
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_repeated_arm_and_invalid_exit_manual_return_conflict():
    async def scenario():
        client, _app, transport, _proxy, _sim = await make_stack()
        try:
            assert (await client.post("/api/arm")).status_code == 200
            repeated = await client.post("/api/arm")
            assert repeated.status_code == 409
            invalid_exit = await client.post("/api/exit-manual")
            assert invalid_exit.status_code == 409
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_pwm_request_rejects_coerced_types_and_unknown_fields():
    async def scenario():
        client, _app, transport, _proxy, _sim = await make_stack()
        try:
            invalid_payloads = [
                {"channel": True, "pwm_us": 1520, "duration_ms": 300},
                {"channel": "0", "pwm_us": 1520, "duration_ms": 300},
                {"channel": 0, "pwm_us": "1520", "duration_ms": 300},
                {"channel": 0, "pwm_us": 1520, "duration_ms": "300"},
                {
                    "channel": 0,
                    "pwm_us": 1520,
                    "duration_ms": 300,
                    "unexpected": "ignored before fix",
                },
            ]
            for payload in invalid_payloads:
                response = await client.post("/api/pwm/test", json=payload)
                assert response.status_code == 422
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_missing_static_asset_is_not_rewritten_to_spa_index(tmp_path):
    async def scenario():
        dist = tmp_path / "dist"
        dist.mkdir()
        (dist / "index.html").write_text("SPA", encoding="utf-8")
        client, _app, transport, _proxy, _sim = await make_stack(
            open_transport=False, frontend_dist=dist)
        try:
            response = await client.get("/assets/missing.js")
            assert response.status_code == 404
            assert response.text != "SPA"
        finally:
            await close_stack(client, transport)

    run(scenario())
