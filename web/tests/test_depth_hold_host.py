import asyncio
import json
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "protocol" / "shared"),
)

from opi_console.config import AppConfig
from opi_console.depth_pid_tuning import (
    DepthPidTuningStore,
    depth_pid_tuning_from_dict,
    validate_depth_pid_tuning,
)
from opi_console.serial_transport import SerialTransport
from opi_console.simulated_stm32 import SimulatedStm32
from opi_console.stm32_proxy import Stm32Proxy
from protocol import DepthPidTuning, NeutralReason, SafetyState


def run(coro):
    return asyncio.run(coro)


async def make_stack(tmp_path: Path, *, control_report_hz: float = 5.0):
    config = AppConfig.model_validate({
        "motion_tuning": {
            "file": str(tmp_path / "motion_tuning.json"),
        },
        "depth_pid_tuning": {
            "file": str(tmp_path / "depth_pid_tuning.json"),
            "sync_interval_s": 0.1,
            "control_report_hz": control_report_hz,
        },
        "telemetry": {
            "status_hz": 20,
            "sensors_hz": 20,
            "request_timeout_s": 0.2,
        },
    })
    transport = SerialTransport(
        heartbeat_interval=0.02,
        heartbeat_timeout_ms=200,
        config=config.transport_dict(),
    )
    transport._ack_timeout = 0.1
    simulator = SimulatedStm32(
        heartbeat_timeout_ms=200,
        status_report_hz=20,
    )
    transport.attach_simulator(simulator)
    proxy = Stm32Proxy(transport, config=config)
    await transport.open()
    await asyncio.sleep(0.05)
    assert await proxy.request_status() is not None
    return transport, proxy, simulator


def test_depth_pid_store_validation_and_capabilities(tmp_path):
    store = DepthPidTuningStore(tmp_path / "depth_pid_tuning.json")
    assert store.load().to_dict() == DepthPidTuning().to_dict()

    tuning = DepthPidTuning(
        kp=15.0,
        ki=0.1,
        kd=5.0,
        p_limit_us=120.0,
        i_limit_us=30.0,
        d_limit_us=25.0,
        output_limit_us=180.0,
    )
    store.save(tuning)
    assert store.load().values() == pytest.approx(tuning.values())
    assert json.loads(store.path.read_text(encoding="utf-8")) == (
        tuning.to_dict())

    with pytest.raises(ValueError, match="output_limit_us"):
        validate_depth_pid_tuning(DepthPidTuning(output_limit_us=0.0))
    with pytest.raises(ValueError, match="booleans"):
        depth_pid_tuning_from_dict({
            **DepthPidTuning().to_dict(),
            "kp": True,
        })

    capabilities = AppConfig().capabilities_dict()
    assert capabilities["features"]["depth_hold"] is True
    assert capabilities["depth_pid"] == {
        "kp_min": 0.0,
        "kp_max": 100.0,
        "ki_min": 0.0,
        "ki_max": 10.0,
        "kd_min": 0.0,
        "kd_max": 100.0,
        "term_limit_min_us": 0.0,
        "term_limit_max_us": 200.0,
        "output_limit_min_us": 1.0,
        "output_limit_max_us": 200.0,
        "target_depth_min_m": 0.0,
        "target_depth_max_m": 300.0,
        "lease_timeout_ms": 500,
    }


def test_proxy_depth_hold_lifecycle_and_pid_diagnostics(tmp_path):
    async def scenario():
        transport, proxy, simulator = await make_stack(tmp_path)
        try:
            ready = await proxy.request_depth_control()
            assert ready is not None
            assert ready.enabled is False
            assert ready.sensor_ready is True
            assert ready.sample_fresh_valid is True
            assert ready.actuator_ready is True

            assert await proxy.arm()
            assert await proxy.enable_depth_hold()
            target_cm = simulator.sensors.depth_m * 100.0 + 20.0

            # The caller owns the 500ms lease and must explicitly refresh it.
            for _ in range(4):
                assert await proxy.set_depth_cm(target_cm)
                await asyncio.sleep(0.12)

            report = await proxy.request_depth_control()
            assert report is not None
            assert report.enabled is True
            assert report.requested_target_cm == pytest.approx(
                target_cm, abs=0.01)
            assert report.error_cm > 0.0
            assert report.p_term_us > 0.0
            assert report.output_us > 0.0
            assert simulator.pwm[1] > 1500
            assert simulator.pwm[2] > 1500
            assert simulator.pwm[5] < 1500
            assert simulator.pwm[6] < 1500

            snapshot = proxy.status_snapshot()
            assert snapshot["depth_pid_tuning_synced"] is True
            assert snapshot["depth_sensor_ready"] is True
            assert snapshot["depth_sample_valid"] is True
            assert snapshot["depth_requested_target_m"] == pytest.approx(
                target_cm / 100.0, abs=1e-4)
            assert snapshot["depth_control"]["enabled"] is True

            assert await proxy.disable_depth_hold()
            status = await proxy.request_status()
            assert status is not None
            assert status.safety_state == SafetyState.ARMED_IDLE
            assert status.float_enabled is False
            assert status.pwm == [1500] * 8
        finally:
            await proxy.stop_background_tasks()
            await transport.close()

    run(scenario())


def test_depth_pid_tuning_write_requires_stopped_state(tmp_path):
    async def scenario():
        transport, proxy, simulator = await make_stack(tmp_path)
        updated = DepthPidTuning(kp=20.0, output_limit_us=150.0)
        try:
            assert await proxy.set_depth_pid_tuning(updated)
            assert simulator.depth_pid_tuning.values() == pytest.approx(
                updated.values())
            assert proxy.depth_pid_tuning_snapshot()["synced"] is True

            assert await proxy.arm()
            assert await proxy.enable_depth_hold()
            rejected = DepthPidTuning(kp=30.0, output_limit_us=140.0)
            assert await proxy.set_depth_pid_tuning(rejected) is False
            assert simulator.depth_pid_tuning.values() == pytest.approx(
                updated.values())
            assert proxy.depth_pid_tuning_snapshot()["sync_state"] == (
                "waiting_for_stop")
        finally:
            await proxy.stop_background_tasks()
            await transport.close()

    run(scenario())


def test_diagnostics_polling_does_not_renew_depth_command_lease(tmp_path):
    async def scenario():
        transport, proxy, simulator = await make_stack(
            tmp_path, control_report_hz=20.0)
        try:
            assert await proxy.arm()
            assert await proxy.enable_depth_hold()
            assert await proxy.set_depth_cm(180.0)
            await proxy.start_background_tasks()

            # REQUEST_DEPTH_CONTROL traffic must not act as SET_DEPTH keepalive.
            await asyncio.sleep(0.65)
            status = await proxy.request_status()
            assert proxy.depth_control_poll_requests >= 5
            assert status is not None
            assert status.safety_state == SafetyState.ARMED_IDLE
            assert status.float_enabled is False
            assert status.pwm == [1500] * 8
            assert simulator.neutral_reason == NeutralReason.COMMAND
        finally:
            await proxy.stop_background_tasks()
            await transport.close()

    run(scenario())


def test_invalid_depth_sample_fails_closed(tmp_path):
    async def scenario():
        transport, proxy, simulator = await make_stack(tmp_path)
        try:
            assert await proxy.arm()
            assert await proxy.enable_depth_hold()
            simulator.depth_sample_valid = False
            assert await proxy.set_depth_cm(180.0) is False
            await asyncio.sleep(0.04)

            status = await proxy.request_status()
            report = await proxy.request_depth_control()
            assert status is not None
            assert report is not None
            assert status.safety_state == SafetyState.ARMED_IDLE
            assert status.float_enabled is False
            assert status.pwm == [1500] * 8
            assert report.sample_fresh_valid is False
            assert report.fault_reason == NeutralReason.DEPTH_SENSOR
        finally:
            await proxy.stop_background_tasks()
            await transport.close()

    run(scenario())


@pytest.mark.parametrize(
    "readiness_fault",
    (
        "sensor_not_ready",
        "sample_invalid",
        "sample_stale",
        "actuator_not_ready",
        "capture_below_range",
        "capture_above_range",
    ),
)
def test_float_on_rejects_missing_depth_readiness(
    tmp_path, readiness_fault
):
    async def scenario():
        case_path = tmp_path / readiness_fault
        transport, proxy, simulator = await make_stack(case_path)
        try:
            assert await proxy.arm()
            if readiness_fault == "sensor_not_ready":
                simulator.depth_sensor_ready = False
            elif readiness_fault == "sample_invalid":
                simulator.depth_sample_valid = False
            elif readiness_fault == "sample_stale":
                simulator.depth_sample_updates_enabled = False
                simulator._depth_sample_at = time.monotonic() - 0.6
            elif readiness_fault == "actuator_not_ready":
                simulator.actuator_output_ready = False
            else:
                depth_m = (
                    -0.1
                    if readiness_fault == "capture_below_range"
                    else 301.0
                )
                simulator.sensors.depth_noise = 0.0
                simulator.sensors._depth_base = depth_m
                simulator.sensors.depth_m = depth_m
                await asyncio.sleep(0.02)

            assert await proxy.enable_depth_hold() is False
            status = await proxy.request_status()
            assert status is not None
            assert status.safety_state == SafetyState.ARMED_IDLE
            assert status.float_enabled is False
            assert status.pwm == [1500] * 8
        finally:
            await proxy.stop_background_tasks()
            await transport.close()

    run(scenario())
