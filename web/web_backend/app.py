"""
FastAPI web backend for the 1132_bot debug console.

Provides REST API and WebSocket telemetry bridging the serial service
to the web frontend.
"""

import logging
import time
from pathlib import Path
from typing import Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from starlette.exceptions import HTTPException as StarletteHTTPException

from opi_console.serial_transport import SerialTransport
from opi_console.stm32_proxy import Stm32Proxy
from opi_console.config import AppConfig, coerce_config
from web_backend.api_routes import create_api_router
from web_backend.ws_manager import WebSocketManager
from web_backend.control_state import ControlState

logger = logging.getLogger("opi_console.web")


class SPAStaticFiles(StaticFiles):
    async def get_response(self, path: str, scope):
        should_fallback = (
            scope.get("method") in ("GET", "HEAD")
            and not Path(path).suffix
        )
        try:
            response = await super().get_response(path, scope)
        except StarletteHTTPException as exc:
            if exc.status_code == 404 and should_fallback:
                return await super().get_response("index.html", scope)
            raise
        if response.status_code == 404 and should_fallback:
            return await super().get_response("index.html", scope)
        return response


def create_app(proxy: Stm32Proxy, transport: SerialTransport,
               config: Optional[AppConfig] = None,
               frontend_dist: Optional[Path] = None) -> FastAPI:
    """Create and configure the FastAPI application.

    Args:
        proxy: The STM32 proxy instance for command dispatch.
        transport: The serial transport for status queries.
    """
    app_config = coerce_config(config or proxy.config)
    control_state = ControlState()
    ws_manager = WebSocketManager(
        proxy, transport, control_state=control_state, config=app_config)
    startup_mono = time.monotonic()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        await proxy.start_background_tasks()
        await ws_manager.start()
        try:
            yield
        finally:
            await ws_manager.stop()
            await proxy.stop_background_tasks()

    app = FastAPI(
        title="1132_bot Debug Console",
        description="Underwater ROV motor calibration and monitoring web console",
        version="1.0.0",
        lifespan=lifespan,
    )
    app.state.config = app_config
    app.state.control_state = control_state
    app.state.ws_manager = ws_manager
    app.state.proxy = proxy

    # CORS (allow web frontend dev server on different port)
    if app_config.web.cors_origins:
        app.add_middleware(
            CORSMiddleware,
            allow_origins=app_config.web.cors_origins,
            allow_credentials=False,
            allow_methods=["GET", "POST", "OPTIONS"],
            allow_headers=["Content-Type"],
        )

    # REST API
    api_router = create_api_router(
        proxy,
        transport,
        ws_manager,
        control_state=control_state,
        config=app_config,
    )
    app.include_router(api_router, prefix="/api")

    # WebSocket endpoint
    @app.websocket("/ws/telemetry")
    async def ws_telemetry(ws: WebSocket):
        await ws_manager.connect(ws)
        try:
            while True:
                data = await ws.receive_text()
                await ws_manager.handle_client_message(ws, data)
        except WebSocketDisconnect:
            await ws_manager.disconnect(ws)
        except Exception:
            logger.exception("WebSocket handler failed")
            await ws_manager.disconnect(ws)

    # Health check
    @app.get("/health")
    async def health():
        state = proxy.refresh_link_state()
        return {
            "status": "ok",
            "mode": "simulation" if transport._sim_stm32 else "hardware",
            "uptime_seconds": time.monotonic() - startup_mono,
            "serial_connected": transport.connected,
            "stm32_online": state.stm32_online,
            "status_stale": state.status_stale,
            "sensors_stale": state.sensors_stale,
            "backend_motion_inhibited": control_state.motion_inhibited,
        }

    # Prevent the root SPA mount from converting unknown API routes to HTML.
    @app.api_route(
        "/api/{path:path}",
        methods=["GET", "HEAD", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"],
        include_in_schema=False,
    )
    async def unknown_api(path: str):
        raise HTTPException(404, f"Unknown API route: /api/{path}")

    # The root static mount must remain last in route registration order.
    static_path = frontend_dist or app_config.resolve_path(
        app_config.web.static_dir)
    if static_path.exists():
        app.mount(
            "/",
            SPAStaticFiles(directory=str(static_path), html=True),
            name="static",
        )
        logger.info("Serving frontend from %s", static_path)

    return app
