import asyncio
import sys
import time
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "protocol" / "shared"))

from opi_console.serial_transport import SerialTransport
from opi_console.stm32_proxy import Stm32Proxy
from opi_console.simulated_stm32 import SimulatedStm32
from web_backend.app import create_app
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
