#!/usr/bin/env python3
"""
1132_bot — Orange Pi Debug Console

Entry point for the serial service, simulator, and web backend.

Usage:
    python -m opi_console.main --simulate
    python -m opi_console.main --serial /dev/ttyS5 --web-port 8000
"""

import argparse
import asyncio
import fcntl
import os
import sys
from contextlib import contextmanager
from pathlib import Path

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "protocol", "shared"))

from opi_console.logger import setup_logging, get_logger
from opi_console.serial_transport import SerialTransport
from opi_console.stm32_proxy import Stm32Proxy
from opi_console.simulated_stm32 import SimulatedStm32
from opi_console.config import AppConfig, load_app_config

logger = get_logger("main")
PROJECT_ROOT = Path(__file__).resolve().parent.parent


class ProcessLockError(RuntimeError):
    pass


def load_config(config_path: str) -> AppConfig:
    """Load configuration from YAML file."""
    return load_app_config(config_path)


def _env_bool(name: str):
    value = os.getenv(name)
    if value is None:
        return None
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise SystemExit(
        f"Invalid {name}={value!r}; expected true/false, 1/0, yes/no, or on/off")


def _env_int(name: str):
    value = os.getenv(name)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError as exc:
        raise SystemExit(f"Invalid {name}={value!r}; expected an integer") from exc


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="1132_bot Debug Console")
    p.add_argument("--config", default=os.getenv(
        "ROV_CONFIG", "opi_console/config.yaml"),
                   help="Path to YAML config file")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--simulate", dest="simulate", action="store_true",
                      help="Run in simulation mode (no real hardware)")
    mode.add_argument("--hardware", dest="simulate", action="store_false",
                      help="Force real hardware mode")
    p.set_defaults(simulate=_env_bool("ROV_SIMULATE"))
    p.add_argument("--serial", default=os.getenv("ROV_SERIAL_PORT"),
                   help="Serial port device (overrides config)")
    p.add_argument("--baud", type=int, default=_env_int("ROV_SERIAL_BAUD"),
                   help="Serial baud rate (overrides config)")
    p.add_argument("--web-host", default=os.getenv("ROV_WEB_HOST"),
                   help="Web server host (overrides config)")
    p.add_argument("--web-port", type=int, default=_env_int("ROV_WEB_PORT"),
                   help="Web server port (overrides config)")
    p.add_argument("--log-level", default=os.getenv("ROV_LOG_LEVEL"),
                   choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
                   help="Log level: DEBUG, INFO, WARNING, ERROR")
    return p.parse_args(argv)


def apply_overrides(config: AppConfig, args) -> AppConfig:
    """Apply CLI/environment overrides and validate the effective config."""
    data = config.model_dump(mode="python")
    if args.serial is not None:
        data["serial"]["port"] = args.serial
    if args.baud is not None:
        data["serial"]["baudrate"] = args.baud
    if args.web_host is not None:
        data["web"]["host"] = args.web_host
    if args.web_port is not None:
        data["web"]["port"] = args.web_port
    if args.log_level is not None:
        data["logging"]["level"] = args.log_level
    if args.simulate is not None:
        data["simulation"]["enabled"] = args.simulate
    return AppConfig.model_validate(data)


@contextmanager
def acquire_process_lock(path: Path):
    """Prevent multiple console processes from controlling one STM32."""
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.seek(0)
            owner = handle.read().strip() or "unknown PID"
            raise ProcessLockError(
                f"Another 1132_bot console instance is running ({owner}); "
                f"lock file: {path}") from exc
        handle.seek(0)
        handle.truncate()
        handle.write(f"pid={os.getpid()}\n")
        handle.flush()
        yield
    finally:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()


async def run_simulation_tick(sim: SimulatedStm32, interval_s: float = 0.01):
    """Periodically tick the simulator."""
    while True:
        try:
            await sim.tick(interval_s)
            await asyncio.sleep(interval_s)
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("Simulator tick task failed; retrying in 1s")
            await asyncio.sleep(1.0)


async def enter_safe_state(proxy: Stm32Proxy, transport: SerialTransport,
                           timeout_s: float, reason: str) -> dict:
    """Best-effort confirmed neutral and DISARM at a process boundary."""
    result = {"neutral": False, "disarm": False}
    if not transport.connected:
        logger.warning("%s safety commands skipped: serial link is offline", reason)
        return result
    try:
        result["neutral"] = await proxy.force_neutral(
            reason, confirm=True, timeout=timeout_s)
    except Exception:
        logger.exception("Unexpected %s neutral failure", reason)
    try:
        result["disarm"] = await asyncio.wait_for(
            proxy.disarm(), timeout=timeout_s)
    except asyncio.TimeoutError:
        logger.error("%s DISARM timed out", reason)
    except Exception:
        logger.exception("Unexpected %s DISARM failure", reason)
    logger.warning(
        "%s safety result neutral=%s disarm=%s",
        reason, result["neutral"], result["disarm"])
    return result


async def enter_safe_shutdown(proxy: Stm32Proxy, transport: SerialTransport,
                              timeout_s: float) -> dict:
    return await enter_safe_state(
        proxy, transport, timeout_s=timeout_s, reason="service_shutdown")


async def run_console(config: AppConfig, args):
    """Main async entry point."""
    sim_task = None
    transport = None
    # ---- Setup logging ----
    log_cfg = config.logging
    log_level = args.log_level or log_cfg.level
    setup_logging(
        log_level=log_level,
        log_file=str(config.resolve_path(log_cfg.file)),
        event_file=str(config.resolve_path(log_cfg.event_file)),
        max_bytes=log_cfg.max_bytes,
        backup_count=log_cfg.backup_count,
    )
    logger.info("Configuration: %s", config.resolve_path(args.config))

    web_cfg = config.web
    static_index = config.resolve_path(web_cfg.static_dir) / "index.html"
    if not static_index.is_file():
        raise RuntimeError(
            f"Frontend build not found: {static_index}. "
            "Run 'cd web_frontend && npm ci && npm run build'.")

    # ---- Determine serial port ----
    serial_cfg = config.serial
    port = serial_cfg.port
    baud = serial_cfg.baudrate
    logger.info("Serial configuration: %s @ %d baud", port, baud)

    hb_cfg = config.heartbeat
    hb_interval = hb_cfg.interval_ms / 1000.0
    hb_timeout = hb_cfg.timeout_ms

    recon_cfg = config.reconnect
    recon_delays_list = []
    delay = recon_cfg.min_delay_s
    while delay < recon_cfg.max_delay_s:
        recon_delays_list.append(delay)
        delay = min(delay * 2, recon_cfg.max_delay_s)
    recon_delays_list.append(recon_cfg.max_delay_s)
    recon_delays = tuple(dict.fromkeys(recon_delays_list))

    # ---- Create transport ----
    transport = SerialTransport(
        port=port,
        baudrate=baud,
        heartbeat_interval=hb_interval,
        heartbeat_timeout_ms=hb_timeout,
        reconnect_delays=recon_delays,
        max_reconnect_retries=recon_cfg.max_retries,
        config=config.transport_dict(),
    )

    sim = None
    if config.simulation.enabled:
        logger.info("=" * 60)
        logger.info("*** SIMULATION MODE — No real hardware connected ***")
        logger.info("=" * 60)

        sim = SimulatedStm32(
            heartbeat_timeout_ms=hb_timeout,
            channel_timeout_ms=config.pwm.default_timeout_ms,
            pwm_neutral=config.pwm.neutral_us,
            pwm_test_min=config.pwm.min_test_us,
            pwm_test_max=config.pwm.max_test_us,
            min_test_duration_ms=config.pwm.min_test_duration_ms,
            max_test_duration_ms=config.pwm.max_test_duration_ms,
            status_report_hz=config.telemetry.status_hz,
            sensor_report_hz=0.0,
        )
        transport.attach_simulator(sim)
        sim_task = asyncio.create_task(run_simulation_tick(sim), name="sim-tick")
        transport._sim_tick_task = sim_task
    else:
        logger.info("=" * 60)
        logger.info("*** REAL HARDWARE MODE — %s @ %d baud ***", port, baud)
        logger.info("=" * 60)

    # ---- Create proxy before opening transport so no early frames are missed ----
    proxy = Stm32Proxy(transport, config=config)

    try:
        # ---- Open transport ----
        await transport.open()
        await enter_safe_state(
            proxy,
            transport,
            timeout_s=config.safety.disconnect_command_timeout_s,
            reason="service_startup",
        )

        # ---- Start web server ----
        web_host = web_cfg.host
        web_port = web_cfg.port
        from web_backend.app import create_app
        app = create_app(proxy, transport, config=config)
        import uvicorn
        server = uvicorn.Server(
            uvicorn.Config(app, host=web_host, port=web_port, log_level="info")
        )
        logger.info("Web console: http://%s:%d", web_host, web_port)
        await server.serve()
    except ImportError as e:
        logger.error("Web backend dependency is unavailable: %s", e)
        raise RuntimeError(
            f"Web backend dependency is unavailable: {e}") from e

    finally:
        logger.info("Shutting down...")
        if proxy is not None and transport is not None:
            await enter_safe_shutdown(
                proxy, transport,
                timeout_s=config.safety.disconnect_command_timeout_s)
        if sim_task:
            sim_task.cancel()
            try:
                await sim_task
            except asyncio.CancelledError:
                pass
        if transport:
            await transport.close()
        logger.info("Done.")


def main():
    args = parse_args()
    try:
        config = apply_overrides(load_config(args.config), args)
    except (ValueError, TypeError) as exc:
        raise SystemExit(f"Configuration error: {exc}") from exc

    try:
        lock_path = config.resolve_path(config.safety.process_lock_file)
        with acquire_process_lock(lock_path):
            asyncio.run(run_console(config, args))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except ProcessLockError as exc:
        raise SystemExit(f"Startup error: {exc}") from exc


if __name__ == "__main__":
    main()
