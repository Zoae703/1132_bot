import asyncio
import sys
import time
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "protocol" / "shared"))

from protocol import DepthControlReport, DepthPidTuning, SafetyState
from opi_console.serial_transport import SerialTransport
from opi_console.stm32_proxy import Stm32Proxy
from opi_console.simulated_stm32 import SimulatedStm32
from web_backend.app import create_app
from web_backend.control_arbiter import ControlMode
from web_backend.ws_manager import WebSocketManager


async def make_stack(open_transport=True, sim_kwargs=None, ack_timeout=0.1):
    sim_kwargs = sim_kwargs or {}
    transport = SerialTransport(heartbeat_interval=0.02, heartbeat_timeout_ms=200)
    transport._ack_timeout = ack_timeout
    sim = SimulatedStm32(heartbeat_timeout_ms=200, **sim_kwargs)
    transport.attach_simulator(sim)
    proxy = Stm32Proxy(transport)
    if open_transport:
        await transport.open()
        await asyncio.sleep(0.05)
        await proxy.request_status()
    app = create_app(proxy, transport)
    client = httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app),
        base_url="http://test",
    )
    return client, transport, proxy, sim


async def close_stack(client, transport):
    await client.aclose()
    await transport.close()


async def enter_manual(client):
    assert (await client.post("/api/arm")).status_code == 200
    assert (await client.post("/api/enter-manual")).status_code == 200


def run(coro):
    return asyncio.run(coro)


def install_depth_route_stubs(proxy):
    calls = []
    tuning = DepthPidTuning()
    report = DepthControlReport(
        requested_target_cm=150.0,
        active_setpoint_cm=150.0,
        measured_depth_cm=150.0,
        sample_age_ms=10,
        flags=0x26,  # sensor ready + valid/fresh sample + actuator ready
    )

    def tuning_snapshot():
        return {
            "desired": tuning.to_dict(),
            "confirmed": tuning.to_dict(),
            "synced": True,
            "sync_state": "synced",
            "sync_error": None,
        }

    async def set_tuning(value, persist=True):
        nonlocal tuning
        calls.append(("set_tuning", value, persist))
        tuning = value
        return True

    async def ensure_tuning():
        calls.append(("ensure_tuning",))
        return True

    async def request_control():
        calls.append(("request_control",))
        return report

    async def enable():
        calls.append(("enable",))
        proxy._state.safety_state = SafetyState.ARMED_ACTIVE
        proxy._state.flags |= 0x03
        report.flags |= 0x01
        return True

    async def set_depth(target_cm):
        calls.append(("set_depth", target_cm))
        report.requested_target_cm = target_cm
        return True

    async def disable():
        calls.append(("disable",))
        proxy._state.safety_state = SafetyState.ARMED_IDLE
        proxy._state.flags &= ~0x03
        proxy._state.pwm = [1500] * 8
        report.flags &= ~0x01
        return True

    proxy.depth_pid_tuning_snapshot = tuning_snapshot
    proxy.set_depth_pid_tuning = set_tuning
    proxy.ensure_depth_pid_tuning_synced = ensure_tuning
    proxy.request_depth_control = request_control
    proxy.enable_depth_hold = enable
    proxy.set_depth_cm = set_depth
    proxy.disable_depth_hold = disable
    return calls, report


def test_depth_hold_routes_confirm_target_and_release_only_after_safe_disable():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        calls, _report = install_depth_route_stubs(proxy)
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200

            tuning = {
                "kp": 8.0,
                "ki": 0.01,
                "kd": 4.0,
                "p_limit_us": 80.0,
                "i_limit_us": 30.0,
                "d_limit_us": 40.0,
                "output_limit_us": 120.0,
            }
            tuned = await client.post("/api/depth/tuning", json=tuning)
            assert tuned.status_code == 200, tuned.text
            assert tuned.json()["confirmed"]["kp"] == 8.0

            enabled = await client.post(
                "/api/depth/enable", json={"target_depth_m": 1.25})
            assert enabled.status_code == 200, enabled.text
            assert enabled.json()["control"]["enabled"] is True
            assert enabled.json()["control"]["depth_requested_target_m"] == 1.25
            assert calls.index(("enable",)) < calls.index(("set_depth", 125.0))

            status = (await client.get("/api/status")).json()
            assert status["control_mode"] == "DEPTH_HOLD"

            target = await client.post(
                "/api/depth/target", json={"target_depth_m": 1.4})
            assert target.status_code == 200, target.text
            assert target.json()["control"]["depth_requested_target_m"] == 1.4

            keepalive = await client.post(
                "/api/depth/keepalive", json={"target_depth_m": 1.4})
            assert keepalive.status_code == 200, keepalive.text
            assert keepalive.json()["target_depth_m"] == 1.4

            disabled = await client.post("/api/depth/disable")
            assert disabled.status_code == 200, disabled.text
            status = (await client.get("/api/status")).json()
            assert status["control_mode"] == "IDLE"
            assert status["safety_state"] == SafetyState.ARMED_IDLE
            assert status["float_enabled"] is False
            assert status["confirmed_pwm"] == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_depth_enable_rejects_unready_diagnostics_and_mode_conflicts():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        calls, report = install_depth_route_stubs(proxy)
        control = client._transport.app.state.control_state
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200

            proxy._state.depth_m = -0.05
            invalid_reference = await client.post(
                "/api/depth/enable", json={})
            assert invalid_reference.status_code == 409
            assert "cannot seed a safe" in invalid_reference.json()["detail"]
            assert ("enable",) not in calls
            proxy._state.depth_m = 1.5

            report.flags &= ~0x20
            unready = await client.post("/api/depth/enable", json={})
            assert unready.status_code == 409
            assert "actuator_ready=False" in unready.json()["detail"]
            assert ("enable",) not in calls

            report.flags |= 0x20
            control.arbiter.acquire(ControlMode.WEB_MOTION, "web")
            conflict = await client.post("/api/depth/enable", json={})
            assert conflict.status_code == 409
            assert "WEB_MOTION" in conflict.json()["detail"]
            assert ("enable",) not in calls
        finally:
            control.arbiter.force_idle("test_cleanup")
            await close_stack(client, transport)

    run(scenario())


def test_depth_enable_target_failure_rolls_back_and_releases_ownership():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        calls, _report = install_depth_route_stubs(proxy)
        control = client._transport.app.state.control_state

        async def reject_target(target_cm):
            calls.append(("set_depth_failed", target_cm))
            return False

        proxy.set_depth_cm = reject_target
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200
            response = await client.post(
                "/api/depth/enable", json={"target_depth_m": 2.0})
            assert response.status_code == 504, response.text
            assert ("disable",) in calls
            assert control.arbiter.mode.value == "IDLE"
            assert proxy.state.safety_state == SafetyState.ARMED_IDLE
            assert proxy.state.pwm == [1500] * 8
            assert control.motion_inhibited is False
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_depth_enable_cancellation_rolls_back_and_releases_ownership():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        calls, report = install_depth_route_stubs(proxy)
        control = client._transport.app.state.control_state
        enable_started = asyncio.Event()

        async def cancellable_enable():
            calls.append(("enable_started",))
            proxy._state.safety_state = SafetyState.ARMED_ACTIVE
            proxy._state.flags |= 0x03
            report.flags |= 0x01
            enable_started.set()
            await asyncio.Event().wait()

        proxy.enable_depth_hold = cancellable_enable
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200
            request = asyncio.create_task(
                client.post("/api/depth/enable", json={}))
            await asyncio.wait_for(enable_started.wait(), 1.0)
            request.cancel()
            try:
                await request
                raise AssertionError("cancelled depth-enable request completed")
            except asyncio.CancelledError:
                pass

            assert ("disable",) in calls
            assert control.arbiter.mode == ControlMode.IDLE
            assert proxy.state.safety_state == SafetyState.ARMED_IDLE
            assert proxy.state.pwm == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_depth_disable_failure_keeps_ownership_and_inhibits_motion():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        _calls, report = install_depth_route_stubs(proxy)
        control = client._transport.app.state.control_state

        async def reject_disable():
            return False

        async def reject_neutral(*_args, **_kwargs):
            return False

        proxy.disable_depth_hold = reject_disable
        proxy.force_neutral = reject_neutral
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200
            proxy._state.safety_state = SafetyState.ARMED_ACTIVE
            proxy._state.flags |= 0x03
            report.flags |= 0x01
            control.arbiter.acquire(ControlMode.DEPTH_HOLD, "web")

            response = await client.post("/api/depth/disable")
            assert response.status_code == 504, response.text
            assert control.arbiter.mode.value == "DEPTH_HOLD"
            assert control.motion_inhibited is True
        finally:
            control.arbiter.force_idle("test_cleanup")
            await close_stack(client, transport)

    run(scenario())


def test_depth_link_loss_releases_backend_ownership():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        control = client._transport.app.state.control_state
        try:
            proxy._state.safety_state = SafetyState.ARMED_ACTIVE
            proxy._state.flags |= 0x03
            control.arbiter.acquire(ControlMode.DEPTH_HOLD, "web")
            transport._connected = False

            message = client._transport.app.state.ws_manager._status_message()
            assert message["payload"]["control_mode"] == "IDLE"
            assert control.arbiter.mode == ControlMode.IDLE
            assert (
                control.arbiter.snapshot()["last_release_reason"]
                == "depth_hold_link_unready"
            )
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_unowned_firmware_depth_hold_can_only_be_stopped():
    async def scenario():
        client, transport, proxy, sim = await make_stack()
        control = client._transport.app.state.control_state
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200
            assert await proxy.enable_depth_hold()
            assert control.arbiter.mode == ControlMode.IDLE
            assert sim.float_enabled is True

            target = await client.post(
                "/api/depth/target", json={"target_depth_m": 2.0})
            keepalive = await client.post(
                "/api/depth/keepalive", json={"target_depth_m": 2.0})
            assert target.status_code == 409
            assert keepalive.status_code == 409

            disabled = await client.post("/api/depth/disable")
            assert disabled.status_code == 200, disabled.text
            assert control.arbiter.mode == ControlMode.IDLE
            assert sim.float_enabled is False
            assert sim.safety_state == SafetyState.ARMED_IDLE
            assert sim.pwm == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_disarm_preempts_and_releases_depth_hold_mode():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        _calls, _report = install_depth_route_stubs(proxy)
        control = client._transport.app.state.control_state
        try:
            assert (await client.post("/api/arm")).status_code == 200
            control.arbiter.acquire(ControlMode.DEPTH_HOLD, "web")
            response = await client.post("/api/disarm")
            assert response.status_code == 200, response.text
            assert control.arbiter.mode.value == "IDLE"
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_estop_preempts_and_releases_depth_hold_mode():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack()
        control = client._transport.app.state.control_state
        try:
            assert (await client.post("/api/arm")).status_code == 200
            control.arbiter.acquire(ControlMode.DEPTH_HOLD, "web")
            response = await client.post("/api/emergency-stop")
            assert response.status_code == 200, response.text
            assert control.arbiter.mode == ControlMode.IDLE
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_firmware_depth_lease_expiry_reconciles_backend_ownership():
    async def scenario():
        client, transport, proxy, sim = await make_stack()
        control = client._transport.app.state.control_state
        try:
            assert await proxy.request_sensors() is not None
            assert (await client.post("/api/arm")).status_code == 200
            enabled = await client.post("/api/depth/enable", json={})
            assert enabled.status_code == 200, enabled.text
            assert control.arbiter.mode == ControlMode.DEPTH_HOLD

            await asyncio.sleep(0.6)
            assert sim.float_enabled is False
            report = await client.get("/api/depth/control")
            assert report.status_code == 200, report.text
            assert report.json()["control"]["enabled"] is False
            assert report.json()["control_mode"] == "IDLE"
            assert control.arbiter.mode == ControlMode.IDLE
            assert sim.safety_state == SafetyState.ARMED_IDLE
            assert sim.pwm == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_unarmed_pwm_request_is_rejected():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack()
        try:
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 409
            assert "DISARMED" in response.json()["detail"]
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_arm_manual_pwm_and_neutral_success():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack()
        try:
            await enter_manual(client)
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 200
            assert response.json()["channel"] == 0

            response = await client.post("/api/pwm/neutral")
            assert response.status_code == 200

            status = (await client.get("/api/status")).json()
            assert status["confirmed_pwm"] == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_pwm_and_duration_limits_are_rejected():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack()
        try:
            await enter_manual(client)
            bad_pwm = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1700, "duration_ms": 300},
            )
            assert bad_pwm.status_code == 422

            bad_duration = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 3000},
            )
            assert bad_duration.status_code == 422
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_estop_rejects_pwm_requests():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack()
        try:
            await enter_manual(client)
            assert (await client.post("/api/emergency-stop")).status_code == 200
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 409
            assert "Emergency stop" in response.json()["detail"]
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_offline_stm32_rejects_pwm_requests():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack(open_transport=False)
        try:
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 503
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_ack_timeout_returns_failure():
    async def scenario():
        client, transport, _proxy, sim = await make_stack(ack_timeout=0.05)
        try:
            await enter_manual(client)
            sim.drop_acks = True
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 504
            assert transport.ack_timeouts >= 1
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_test_duration_eventually_returns_to_neutral():
    async def scenario():
        client, transport, proxy, sim = await make_stack()
        try:
            await enter_manual(client)
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 2, "pwm_us": 1530, "duration_ms": 200},
            )
            assert response.status_code == 200
            await asyncio.sleep(0.25)
            await sim.tick(0.25)
            await proxy.request_status()
            assert proxy.state.pwm == [1500] * 8
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_concurrent_two_channel_pwm_does_not_allow_both_requests():
    async def scenario():
        client, transport, _proxy, _sim = await make_stack()
        try:
            await enter_manual(client)
            responses = await asyncio.gather(
                client.post("/api/pwm/test", json={"channel": 0, "pwm_us": 1520, "duration_ms": 300}),
                client.post("/api/pwm/test", json={"channel": 1, "pwm_us": 1525, "duration_ms": 300}),
            )
            assert sum(1 for response in responses if response.status_code == 200) <= 1
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_stale_status_marks_offline_and_disables_control():
    async def scenario():
        client, transport, proxy, _sim = await make_stack()
        try:
            await proxy.request_status()
            transport.last_rx_time = time.time() - 5.0
            transport._last_any_frame_mono = time.monotonic() - 5.0
            status = (await client.get("/api/status")).json()
            assert status["stm32_online"] is False
            assert status["status_stale"] is True
            response = await client.post(
                "/api/pwm/test",
                json={"channel": 0, "pwm_us": 1520, "duration_ms": 300},
            )
            assert response.status_code == 503
        finally:
            await close_stack(client, transport)

    run(scenario())


def test_websocket_broadcast_removes_dead_clients():
    class FakeWebSocket:
        def __init__(self, fail=False):
            self.fail = fail
            self.messages = []

        async def accept(self):
            pass

        async def close(self):
            pass

        async def send_text(self, data):
            if self.fail:
                raise RuntimeError("client disconnected")
            self.messages.append(data)

    async def scenario():
        _client, transport, proxy, _sim = await make_stack()
        manager = WebSocketManager(proxy, transport)
        live = FakeWebSocket()
        dead = FakeWebSocket(fail=True)
        try:
            await manager.connect(live)
            await manager.connect(dead)
            await manager._broadcast_status()
            assert len(live.messages) == 1
            assert manager.client_count == 1
        finally:
            await manager.stop()
            await close_stack(_client, transport)

    run(scenario())
