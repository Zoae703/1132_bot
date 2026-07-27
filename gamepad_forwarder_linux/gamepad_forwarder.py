#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import pygame
import tkinter as tk
from tkinter import messagebox, ttk
import websocket

from gamepad_mapping import (
    MappedCommand,
    MappingConfig,
    map_gamepad,
)

APP_NAME = "gamepad-forwarder"
CONFIG_DIR = Path.home() / ".config" / APP_NAME
CONFIG_FILE = CONFIG_DIR / "settings.json"


@dataclass
class Config:
    ip: str = "192.168.127.10"
    port: int = 8000
    path: str = "/ws/control/gamepad"
    hz: float = 50.0


@dataclass
class Status:
    running: bool = False
    gamepad_connected: bool = False
    gamepad_name: str = "未连接"
    server_connected: bool = False
    control_enabled: bool = False
    sequence: int = 0
    actual_hz: float = 0.0
    axes: tuple[float, ...] = ()
    buttons: tuple[int, ...] = ()
    pressed_buttons: tuple[int, ...] = ()
    hats: tuple[tuple[int, int], ...] = ()
    mapped_command: tuple[float, ...] = (0.0,) * 6
    heave_conflict: bool = False
    ack_sequence: int | None = None
    ack_accepted: bool = False
    ack_reason: str = "尚未收到"
    ack_rtt_ms: float | None = None
    server_control_mode: str = "IDLE"
    mapping_summary: str = "等待服务端配置"
    last_error: str = ""


def load_config() -> Config:
    try:
        data = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        return Config(
            ip=str(data.get("ip", Config.ip)),
            port=int(data.get("port", Config.port)),
            path=str(data.get("path", Config.path)),
            hz=float(data.get("hz", Config.hz)),
        )
    except FileNotFoundError:
        return Config()
    except Exception:
        return Config()


def save_config(config: Config) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    temp = CONFIG_FILE.with_suffix(".tmp")
    temp.write_text(
        json.dumps(asdict(config), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temp, CONFIG_FILE)


class SenderWorker:
    """后台读取手柄并通过 WebSocket 发送。"""

    def __init__(self) -> None:
        self._stop = threading.Event()
        self._running = threading.Event()
        self._control = threading.Event()
        self._lock = threading.Lock()
        self._config = Config()
        self._status = Status()
        self._thread = threading.Thread(target=self._main, daemon=True)
        self._thread.start()

    def configure(self, config: Config) -> None:
        with self._lock:
            self._config = Config(**asdict(config))

    def start(self) -> None:
        self._running.set()
        self._set_status(running=True)

    def stop(self) -> None:
        self._control.clear()
        self._running.clear()
        self._set_status(
            running=False,
            control_enabled=False,
            server_connected=False,
        )

    def set_control(self, enabled: bool) -> None:
        if enabled:
            self._control.set()
        else:
            self._control.clear()
        self._set_status(control_enabled=enabled)

    def snapshot(self) -> Status:
        with self._lock:
            return Status(**asdict(self._status))

    def close(self) -> None:
        self._control.clear()
        self._running.clear()
        self._stop.set()
        self._thread.join(timeout=3)

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            for key, value in changes.items():
                setattr(self._status, key, value)

    def _get_config(self) -> Config:
        with self._lock:
            return Config(**asdict(self._config))

    def _main(self) -> None:
        pygame.init()
        pygame.joystick.init()
        joystick: pygame.joystick.Joystick | None = None
        ws: websocket.WebSocket | None = None
        sequence = 0
        mapping_config = MappingConfig()

        try:
            while not self._stop.is_set():
                joystick = self._refresh_gamepad(joystick)

                if not self._running.is_set():
                    self._close_socket(ws)
                    ws = None
                    time.sleep(0.1)
                    continue

                config = self._get_config()
                url = self._url(config)

                if ws is None:
                    try:
                        ws = websocket.create_connection(
                            url,
                            timeout=3,
                            enable_multithread=True,
                        )
                        ws.settimeout(2)
                        hello = self._receive_json(ws)
                        if (
                            hello.get("type") != "gamepad_hello"
                            or not hello.get("lease_granted")
                        ):
                            raise RuntimeError(
                                f"控制租约被拒绝：{hello.get('reason', 'unknown')}")
                        mapping_config = MappingConfig.from_server(
                            hello.get("mapping_config", {}))
                        session_id = uuid.uuid4().hex
                        sequence = 0
                        ws.settimeout(1)
                        self._set_status(
                            server_connected=True,
                            server_control_mode="IDLE",
                            mapping_summary=self._mapping_summary(
                                mapping_config),
                            last_error="",
                        )
                    except Exception as exc:
                        self._control.clear()
                        self._set_status(
                            server_connected=False,
                            control_enabled=False,
                            last_error=f"连接失败：{exc}",
                        )
                        self._close_socket(ws)
                        ws = None
                        time.sleep(1)
                        continue

                started = time.perf_counter()
                sent = 0
                period = 1.0 / config.hz
                next_send = time.perf_counter()

                try:
                    while self._running.is_set() and not self._stop.is_set():
                        joystick = self._refresh_gamepad(joystick)
                        state = self._read_gamepad(joystick)
                        layout_ok = (
                            state["connected"]
                            and len(state["axes"]) == mapping_config.axis_count
                            and mapping_config.min_button_count
                            <= len(state["buttons"])
                            <= mapping_config.max_button_count
                        )
                        if layout_ok:
                            packet_axes = state["axes"]
                            packet_buttons = state["buttons"]
                        else:
                            packet_axes = [0.0] * mapping_config.axis_count
                            packet_buttons = [0] * mapping_config.min_button_count

                        if state["connected"] and not layout_ok:
                            self._control.clear()
                            layout_error = (
                                "手柄字段不匹配：需要6轴、4至32个按钮；"
                                f"当前{len(state['axes'])}轴、"
                                f"{len(state['buttons'])}个按钮"
                            )
                        else:
                            layout_error = ""

                        mapped = map_gamepad(
                            packet_axes, packet_buttons, mapping_config)
                        enabled = self._control.is_set() and layout_ok

                        sequence += 1

                        packet = {
                            "type": "gamepad_state",
                            "version": 1,
                            "session_id": session_id,
                            "sequence": sequence,
                            "client_time_ns": time.monotonic_ns(),
                            "control_enabled": enabled,
                            "gamepad_connected": layout_ok,
                            "device": state["device"],
                            "axes": packet_axes,
                            "buttons": packet_buttons,
                            "hats": state["hats"][:8],
                            "mapped_command": mapped.command.as_dict(),
                        }
                        sent_at = time.perf_counter()
                        ws.send(json.dumps(packet, ensure_ascii=False, separators=(",", ":")))
                        ack = self._receive_json(ws)
                        if (
                            ack.get("type") != "gamepad_ack"
                            or ack.get("sequence") != sequence
                        ):
                            raise RuntimeError("收到无效或错序的服务端 ACK")
                        rtt_ms = (time.perf_counter() - sent_at) * 1000.0

                        sent += 1
                        elapsed = time.perf_counter() - started
                        actual_hz = sent / elapsed if elapsed > 0 else 0.0
                        self._set_status(
                            running=True,
                            gamepad_connected=state["connected"],
                            gamepad_name=state["name"],
                            server_connected=True,
                            control_enabled=enabled,
                            sequence=sequence,
                            actual_hz=actual_hz,
                            axes=tuple(state["axes"]),
                            buttons=tuple(state["buttons"]),
                            pressed_buttons=tuple(i for i, value in enumerate(state["buttons"]) if value),
                            hats=tuple(tuple(item) for item in state["hats"]),
                            mapped_command=self._command_tuple(
                                mapped.command),
                            heave_conflict=mapped.heave_conflict,
                            ack_sequence=sequence,
                            ack_accepted=bool(ack.get("accepted")),
                            ack_reason=str(ack.get("reason", "unknown")),
                            ack_rtt_ms=rtt_ms,
                            server_control_mode=str(
                                ack.get("control_mode", "UNKNOWN")),
                            last_error=layout_error,
                        )

                        next_send += period
                        delay = next_send - time.perf_counter()
                        if delay < -period:
                            next_send = time.perf_counter()
                            delay = 0.0
                        time.sleep(max(0.0, delay))

                except Exception as exc:
                    self._control.clear()
                    self._set_status(
                        server_connected=False,
                        control_enabled=False,
                        last_error=f"发送中断：{exc}",
                    )
                    self._close_socket(ws)
                    ws = None
                    time.sleep(1)

        finally:
            self._close_socket(ws)
            if joystick is not None:
                joystick.quit()
            pygame.joystick.quit()
            pygame.quit()

    def _refresh_gamepad(
        self,
        joystick: pygame.joystick.Joystick | None,
    ) -> pygame.joystick.Joystick | None:
        pygame.event.pump()

        if pygame.joystick.get_count() == 0:
            if joystick is not None:
                joystick.quit()
            self._control.clear()
            self._set_status(
                gamepad_connected=False,
                gamepad_name="未连接",
                control_enabled=False,
                axes=(),
                buttons=(),
                pressed_buttons=(),
                hats=(),
                mapped_command=(0.0,) * 6,
                heave_conflict=False,
            )
            return None

        if joystick is not None and joystick.get_init():
            self._set_status(
                gamepad_connected=True,
                gamepad_name=joystick.get_name(),
                last_error="",
            )
            return joystick

        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            self._set_status(
                gamepad_connected=True,
                gamepad_name=joystick.get_name(),
                last_error="",
            )
            return joystick
        except pygame.error as exc:
            self._set_status(last_error=f"手柄打开失败：{exc}")
            return None

    def _read_gamepad(self, joystick: pygame.joystick.Joystick | None) -> dict[str, Any]:
        if joystick is None or not joystick.get_init():
            return {
                "connected": False,
                "name": "未连接",
                "device": None,
                "axes": [],
                "buttons": [],
                "hats": [],
            }

        axes = [self._axis(joystick.get_axis(i)) for i in range(joystick.get_numaxes())]
        buttons = [int(bool(joystick.get_button(i))) for i in range(joystick.get_numbuttons())]
        hats = [list(joystick.get_hat(i)) for i in range(joystick.get_numhats())]
        return {
            "connected": True,
            "name": joystick.get_name(),
            "device": {
                "name": joystick.get_name(),
                "guid": joystick.get_guid(),
                "instance_id": joystick.get_instance_id(),
            },
            "axes": axes,
            "buttons": buttons,
            "hats": hats,
        }

    @staticmethod
    def _axis(value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("手柄轴包含 NaN 或 Inf")
        return round(max(-1.0, min(1.0, float(value))), 5)

    @staticmethod
    def _url(config: Config) -> str:
        path = config.path if config.path.startswith("/") else "/" + config.path
        return f"ws://{config.ip}:{config.port}{path}"

    @staticmethod
    def _receive_json(ws: websocket.WebSocket) -> dict[str, Any]:
        raw = ws.recv()
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8")
        message = json.loads(raw)
        if not isinstance(message, dict):
            raise RuntimeError("服务端消息不是 JSON 对象")
        return message

    @staticmethod
    def _command_tuple(command: MappedCommand) -> tuple[float, ...]:
        return (
            command.surge,
            command.sway,
            command.heave,
            command.roll,
            command.pitch,
            command.yaw,
        )

    @staticmethod
    def _mapping_summary(config: MappingConfig) -> str:
        signs = (
            f"surge反向={'是' if config.surge_invert else '否'}, "
            f"sway反向={'是' if config.sway_invert else '否'}, "
            f"yaw反向={'是' if config.yaw_invert else '否'}"
        )
        return (
            f"死区 {config.deadzone:.2f}, expo {config.expo:.2f}, "
            f"全局倍率 {config.global_scale:.2f}; {signs}"
        )

    @staticmethod
    def _close_socket(ws: websocket.WebSocket | None) -> None:
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("手柄网络转发器")
        self.geometry("800x820")
        self.minsize(720, 700)

        self._setup_styles()

        config = load_config()
        self.worker = SenderWorker()
        self.worker.configure(config)

        self.ip_var = tk.StringVar(value=config.ip)
        self.port_var = tk.StringVar(value=str(config.port))
        self.path_var = tk.StringVar(value=config.path)
        self.hz_var = tk.StringVar(value=str(config.hz))
        self.control_var = tk.BooleanVar(value=False)

        self.gamepad_var = tk.StringVar(value="未连接")
        self.server_var = tk.StringVar(value="未连接")
        self.mode_var = tk.StringVar(value="已停止")
        self.rate_var = tk.StringVar(value="0.0 Hz")
        self.sequence_var = tk.StringVar(value="0")
        self.hats_var = tk.StringVar(value="-")
        self.buttons_var = tk.StringVar(value="-")
        self.mapped_var = tk.StringVar(
            value="surge +0.000  sway +0.000  heave +0.000  yaw +0.000")
        self.ack_var = tk.StringVar(value="尚未收到")
        self.rtt_var = tk.StringVar(value="-")
        self.mapping_var = tk.StringVar(value="等待服务端配置")

        # Status indicator canvases
        self.gamepad_indicator: tk.Canvas | None = None
        self.server_indicator: tk.Canvas | None = None
        self.mode_indicator: tk.Canvas | None = None
        self.status_bar_indicator: tk.Canvas | None = None

        # Container widget references
        self.axes_container: ttk.Frame | None = None
        self.buttons_container: ttk.Frame | None = None
        self.status_bar_label: ttk.Label | None = None
        self.gamepad_label: ttk.Label | None = None
        self.server_label: ttk.Label | None = None
        self.mode_label: ttk.Label | None = None
        self.hats_label: ttk.Label | None = None

        # Axis bar state
        self._axis_bars: list[dict] = []
        self._axes_placeholder: ttk.Label | None = None

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(100, self._refresh)

    # ------------------------------------------------------------------
    # Style helpers
    # ------------------------------------------------------------------

    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.configure("Status.TLabel", font=("TkDefaultFont", 10))
        style.configure("Info.TLabel", foreground="#757575", font=("TkDefaultFont", 9))
        style.configure("Section.TLabel", font=("TkDefaultFont", 10, "bold"))
        style.configure("Error.TLabel", foreground="#F44336")
        style.configure("Ready.TLabel", foreground="#9E9E9E")

    # ------------------------------------------------------------------
    # Status indicator helpers (Canvas-based colored dots)
    # ------------------------------------------------------------------

    @staticmethod
    def _make_indicator(parent: tk.Misc, size: int = 14) -> tk.Canvas:
        canvas = tk.Canvas(
            parent, width=size, height=size,
            highlightthickness=0, bd=0,
        )
        canvas.create_oval(1, 1, size - 1, size - 1, fill="#9E9E9E", outline="", tags="dot")
        return canvas

    @staticmethod
    def _set_indicator(canvas: tk.Canvas, state: str) -> None:
        colors = {
            "good": "#4CAF50",
            "bad": "#F44336",
            "warn": "#FF9800",
            "off": "#9E9E9E",
        }
        canvas.itemconfig("dot", fill=colors.get(state, "#9E9E9E"))

    # ------------------------------------------------------------------
    # Axis progress bars
    # ------------------------------------------------------------------

    def _create_axis_bar(self, parent: ttk.Frame, axis_index: int) -> dict:
        frame = ttk.Frame(parent)

        lbl_name = ttk.Label(frame, text=f"轴 {axis_index}", width=7, anchor=tk.W)
        lbl_name.pack(side=tk.LEFT, padx=(0, 6))

        canvas = tk.Canvas(frame, height=20, highlightthickness=0, bd=0, bg="#E8E8E8")
        canvas.pack(side=tk.LEFT, fill=tk.X, expand=True)

        lbl_value = ttk.Label(frame, text="+0.000", width=8, anchor=tk.E,
                              font=("TkFixedFont", 9))
        lbl_value.pack(side=tk.RIGHT, padx=(6, 0))

        # Pre-create tagged items that will be repositioned on update
        canvas.create_rectangle(0, 0, 0, 0, fill="#BDBDBD", outline="", tags="center")
        canvas.create_rectangle(0, 0, 0, 0, fill="#4CAF50", outline="", tags="fill")

        return {"frame": frame, "canvas": canvas, "value_label": lbl_value}

    def _update_axis_bar(self, bar_info: dict, value: float) -> None:
        canvas: tk.Canvas = bar_info["canvas"]
        value_label: ttk.Label = bar_info["value_label"]

        w = canvas.winfo_width()
        if w < 20:
            value_label.configure(text=f"{value:+.3f}")
            return

        center_x = w / 2
        bar_width = (value / 2.0) * w  # -w/2 (full left) .. +w/2 (full right)

        # zero-line
        canvas.coords("center", center_x, 0, center_x, 22)

        if value >= 0:
            canvas.coords("fill", center_x, 2, center_x + bar_width, 20)
            canvas.itemconfig("fill", fill="#4CAF50")
        else:
            canvas.coords("fill", center_x + bar_width, 2, center_x, 20)
            canvas.itemconfig("fill", fill="#2196F3")

        value_label.configure(text=f"{value:+.3f}")

    def _update_axes(self, axes: tuple[float, ...]) -> None:
        if not axes:
            if self._axis_bars:
                for b in self._axis_bars:
                    b["frame"].destroy()
                self._axis_bars.clear()
            if self._axes_placeholder is None:
                self._axes_placeholder = ttk.Label(
                    self.axes_container, text="无", foreground="#9E9E9E",
                )
                self._axes_placeholder.pack(anchor=tk.W)
            return

        # Remove placeholder if present
        if self._axes_placeholder is not None:
            self._axes_placeholder.destroy()
            self._axes_placeholder = None

        # Rebuild bars if count changed
        if len(axes) != len(self._axis_bars):
            for b in self._axis_bars:
                b["frame"].destroy()
            self._axis_bars.clear()
            for i in range(len(axes)):
                bar = self._create_axis_bar(self.axes_container, i)
                bar["frame"].pack(fill=tk.X, pady=2)
                self._axis_bars.append(bar)

        for bar_info, val in zip(self._axis_bars, axes):
            self._update_axis_bar(bar_info, val)

    # ------------------------------------------------------------------
    # Button chips
    # ------------------------------------------------------------------

    def _update_button_chips(self, buttons: tuple[int, ...]) -> None:
        for child in self.buttons_container.winfo_children():
            child.destroy()

        if not buttons:
            ttk.Label(
                self.buttons_container, text="无", foreground="#9E9E9E",
            ).pack(anchor=tk.W)
            return

        MAX_COL = 10
        for i, btn_num in enumerate(sorted(buttons)):
            chip = tk.Frame(
                self.buttons_container, bg="#E3F2FD", bd=0,
                highlightthickness=1, highlightbackground="#90CAF9",
            )
            chip.grid(row=i // MAX_COL, column=i % MAX_COL, padx=2, pady=2)
            tk.Label(
                chip, text=str(btn_num), bg="#E3F2FD", fg="#1565C0",
                font=("TkDefaultFont", 9, "bold"), padx=4, pady=1,
            ).pack()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # -- Status bar (packed BOTTOM so it reserves space first) --
        status_bar = ttk.Frame(self, relief=tk.SUNKEN, padding=(10, 4))
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        self.status_bar_indicator = self._make_indicator(status_bar, size=12)
        self.status_bar_indicator.pack(side=tk.LEFT, padx=(0, 6))

        self.status_bar_label = ttk.Label(status_bar, text="就绪", style="Ready.TLabel")
        self.status_bar_label.pack(side=tk.LEFT)

        # -- Main content frame --
        main = ttk.Frame(self, padding=14)
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # == Settings ==
        settings = ttk.LabelFrame(main, text="转发设置", padding=14)
        settings.pack(fill=tk.X)

        ttk.Label(settings, text="香橙派 IP").grid(
            row=0, column=0, sticky=tk.W, pady=4,
        )
        ttk.Entry(settings, textvariable=self.ip_var, width=24).grid(
            row=0, column=1, sticky=tk.EW, padx=8, pady=4,
        )
        ttk.Label(settings, text="端口").grid(
            row=0, column=2, sticky=tk.W, pady=4,
        )
        ttk.Entry(settings, textvariable=self.port_var, width=9).grid(
            row=0, column=3, sticky=tk.W, padx=8, pady=4,
        )

        ttk.Label(settings, text="WebSocket 路径").grid(
            row=1, column=0, sticky=tk.W, pady=4,
        )
        ttk.Entry(settings, textvariable=self.path_var).grid(
            row=1, column=1, sticky=tk.EW, padx=8, pady=4,
        )
        ttk.Label(settings, text="发送频率 (Hz)").grid(
            row=1, column=2, sticky=tk.W, pady=4,
        )
        ttk.Spinbox(
            settings, from_=20, to=100, increment=5,
            textvariable=self.hz_var, width=7,
        ).grid(row=1, column=3, sticky=tk.W, padx=8, pady=4)
        settings.columnconfigure(1, weight=1)

        # == Button bar ==
        btn_bar = ttk.Frame(main)
        btn_bar.pack(fill=tk.X, pady=(10, 6))

        btn_left = ttk.Frame(btn_bar)
        btn_left.pack(side=tk.LEFT)

        self.start_button = ttk.Button(
            btn_left, text="启动连接", command=self._start)
        self.start_button.pack(side=tk.LEFT)

        self.stop_button = ttk.Button(
            btn_left, text="停止连接", command=self._stop, state=tk.DISABLED,
        )
        self.stop_button.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(
            btn_left, text="保存设置", command=self._save).pack(side=tk.LEFT)

        ttk.Separator(btn_bar, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=12,
        )

        btn_right = ttk.Frame(btn_bar)
        btn_right.pack(side=tk.RIGHT)
        ttk.Checkbutton(
            btn_right, text="控制开启", variable=self.control_var,
            command=self._toggle_control,
        ).pack(side=tk.RIGHT)

        # == Info line ==
        ttk.Label(
            main,
            text=(
                "控制开关不会 ARM 或解除急停；进入 GAMEPAD 模式必须在网页端完成。"
                "原始输入始终上传，运动命令由香橙派再次映射和校验。"
            ),
            style="Info.TLabel", wraplength=630, justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(0, 10))

        # == Status frame ==
        status = ttk.LabelFrame(main, text="连接状态", padding=14)
        status.pack(fill=tk.X, pady=(0, 10))

        # Row 0: USB Gamepad
        tk_canvas = self._make_indicator(status)
        tk_canvas.grid(row=0, column=0, sticky=tk.W, pady=4, padx=(0, 6))
        ttk.Label(status, text="USB 手柄", style="Status.TLabel").grid(
            row=0, column=1, sticky=tk.W, pady=4,
        )
        self.gamepad_label = ttk.Label(status, textvariable=self.gamepad_var, style="Status.TLabel")
        self.gamepad_label.grid(row=0, column=2, sticky=tk.W, padx=12, pady=4)
        self.gamepad_indicator = tk_canvas

        # Row 1: Server
        tk_canvas = self._make_indicator(status)
        tk_canvas.grid(row=1, column=0, sticky=tk.W, pady=4, padx=(0, 6))
        ttk.Label(status, text="香橙派服务器", style="Status.TLabel").grid(
            row=1, column=1, sticky=tk.W, pady=4,
        )
        self.server_label = ttk.Label(status, textvariable=self.server_var, style="Status.TLabel")
        self.server_label.grid(row=1, column=2, sticky=tk.W, padx=12, pady=4)
        self.server_indicator = tk_canvas

        # Row 2: Work mode
        tk_canvas = self._make_indicator(status)
        tk_canvas.grid(row=2, column=0, sticky=tk.W, pady=4, padx=(0, 6))
        ttk.Label(status, text="工作状态", style="Status.TLabel").grid(
            row=2, column=1, sticky=tk.W, pady=4,
        )
        self.mode_label = ttk.Label(status, textvariable=self.mode_var, style="Status.TLabel")
        self.mode_label.grid(row=2, column=2, sticky=tk.W, padx=12, pady=4)
        self.mode_indicator = tk_canvas

        # Row 3: Rate + Sequence
        ttk.Label(status, text="").grid(row=3, column=0, pady=2)
        ttk.Label(status, text="发送频率", style="Status.TLabel").grid(
            row=3, column=1, sticky=tk.W, pady=4,
        )
        rate_frame = ttk.Frame(status)
        rate_frame.grid(row=3, column=2, sticky=tk.W, padx=12, pady=4)
        ttk.Label(rate_frame, textvariable=self.rate_var, style="Status.TLabel").pack(side=tk.LEFT)
        ttk.Label(rate_frame, text="    最新序号: ").pack(side=tk.LEFT)
        ttk.Label(rate_frame, textvariable=self.sequence_var).pack(side=tk.LEFT)

        ttk.Label(status, text="").grid(row=4, column=0, pady=2)
        ttk.Label(status, text="最近 ACK", style="Status.TLabel").grid(
            row=4, column=1, sticky=tk.W, pady=4,
        )
        ack_frame = ttk.Frame(status)
        ack_frame.grid(row=4, column=2, sticky=tk.W, padx=12, pady=4)
        ttk.Label(
            ack_frame, textvariable=self.ack_var,
            style="Status.TLabel",
        ).pack(side=tk.LEFT)
        ttk.Label(ack_frame, text="    RTT: ").pack(side=tk.LEFT)
        ttk.Label(ack_frame, textvariable=self.rtt_var).pack(side=tk.LEFT)

        ttk.Label(status, text="").grid(row=5, column=0, pady=2)
        ttk.Label(status, text="服务端映射", style="Status.TLabel").grid(
            row=5, column=1, sticky=tk.W, pady=4,
        )
        ttk.Label(
            status,
            textvariable=self.mapping_var,
            style="Info.TLabel",
            wraplength=520,
            justify=tk.LEFT,
        ).grid(row=5, column=2, sticky=tk.W, padx=12, pady=4)

        status.columnconfigure(2, weight=1)

        # == Data frame ==
        data = ttk.LabelFrame(main, text="手柄数据", padding=14)
        data.pack(fill=tk.BOTH, expand=True)

        # Axes section
        ttk.Label(data, text="摇杆 (Axes)", style="Section.TLabel").pack(
            anchor=tk.W, pady=(0, 4),
        )
        self.axes_container = ttk.Frame(data)
        self.axes_container.pack(fill=tk.X, pady=(0, 8))
        # Initialize placeholder
        self._axes_placeholder = ttk.Label(
            self.axes_container, text="等待手柄连接…", foreground="#9E9E9E",
        )
        self._axes_placeholder.pack(anchor=tk.W)

        ttk.Label(data, text="映射后的 BodyCommand", style="Section.TLabel").pack(
            anchor=tk.W, pady=(4, 4),
        )
        ttk.Label(
            data,
            textvariable=self.mapped_var,
            font=("TkFixedFont", 10),
        ).pack(anchor=tk.W, pady=(0, 8))

        ttk.Separator(data, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)

        # Buttons section
        ttk.Label(data, text="按下按钮", style="Section.TLabel").pack(
            anchor=tk.W, pady=(4, 4),
        )
        self.buttons_container = ttk.Frame(data)
        self.buttons_container.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(
            data,
            textvariable=self.buttons_var,
            style="Info.TLabel",
            wraplength=700,
        ).pack(anchor=tk.W, pady=(0, 8))

        ttk.Separator(data, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)

        # Hats section
        ttk.Label(data, text="方向帽 (Hats)", style="Section.TLabel").pack(
            anchor=tk.W, pady=(4, 4),
        )
        self.hats_label = ttk.Label(data, textvariable=self.hats_var)
        self.hats_label.pack(anchor=tk.W, pady=(0, 4))

    def _config_from_ui(self) -> Config:
        ip = self.ip_var.get().strip()
        path = self.path_var.get().strip()
        if not ip:
            raise ValueError("IP 不能为空")
        if not path:
            raise ValueError("WebSocket 路径不能为空")
        try:
            port = int(self.port_var.get())
            hz = float(self.hz_var.get())
        except ValueError as exc:
            raise ValueError("端口和发送频率必须是数字") from exc
        if not 1 <= port <= 65535:
            raise ValueError("端口必须在 1～65535 之间")
        if not 20 <= hz <= 100:
            raise ValueError("发送频率必须在 20～100 Hz 之间")
        return Config(ip=ip, port=port, path=path, hz=hz)

    def _save(self) -> None:
        try:
            config = self._config_from_ui()
            save_config(config)
            self.worker.configure(config)
            messagebox.showinfo("保存完成", f"配置已保存到：\n{CONFIG_FILE}")
        except Exception as exc:
            messagebox.showerror("保存失败", str(exc))

    def _start(self) -> None:
        try:
            config = self._config_from_ui()
        except ValueError as exc:
            messagebox.showerror("启动失败", str(exc))
            return
        self.worker.configure(config)
        self.worker.start()
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)

    def _stop(self) -> None:
        self.control_var.set(False)
        self.worker.set_control(False)
        self.worker.stop()
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)

    def _toggle_control(self) -> None:
        self.worker.set_control(self.control_var.get())

    def _refresh(self) -> None:
        status = self.worker.snapshot()
        if (
            self.control_var.get()
            and not status.control_enabled
            and (
                not status.server_connected
                or not status.gamepad_connected
                or bool(status.last_error)
            )
        ):
            self.control_var.set(False)

        # -- USB Gamepad --
        if status.gamepad_connected:
            self.gamepad_var.set(status.gamepad_name)
            self._set_indicator(self.gamepad_indicator, "good")
        else:
            self.gamepad_var.set("未连接")
            self._set_indicator(self.gamepad_indicator, "bad")

        # -- Server --
        if status.server_connected:
            self.server_var.set("已连接")
            self._set_indicator(self.server_indicator, "good")
        else:
            self.server_var.set("未连接")
            self._set_indicator(self.server_indicator, "bad")

        # -- Work mode --
        if not status.running:
            self.mode_var.set("已停止")
            self._set_indicator(self.mode_indicator, "off")
        elif status.control_enabled:
            self.mode_var.set(
                f"控制已开启 / 服务模式 {status.server_control_mode}")
            self._set_indicator(self.mode_indicator, "good")
        else:
            self.mode_var.set(
                f"控制已关闭 / 服务模式 {status.server_control_mode}")
            self._set_indicator(self.mode_indicator, "warn")

        # -- Rate and sequence --
        self.rate_var.set(f"{status.actual_hz:.1f} Hz")
        self.sequence_var.set(str(status.sequence))
        self.ack_var.set(
            f"seq={status.ack_sequence} "
            f"{'接收' if status.ack_accepted else '拒绝'}: "
            f"{status.ack_reason}"
        )
        self.rtt_var.set(
            f"{status.ack_rtt_ms:.1f} ms"
            if status.ack_rtt_ms is not None else "-"
        )
        self.mapping_var.set(status.mapping_summary)

        # -- Axes (progress bars) --
        self._update_axes(status.axes)

        # -- Buttons (chips) --
        self._update_button_chips(status.pressed_buttons)
        self.buttons_var.set(
            "原始 buttons: "
            + (str(list(status.buttons)) if status.buttons else "无")
        )

        values = status.mapped_command
        self.mapped_var.set(
            f"surge {values[0]:+0.3f}  sway {values[1]:+0.3f}  "
            f"heave {values[2]:+0.3f}  yaw {values[5]:+0.3f}"
            + ("  A/Y 冲突" if status.heave_conflict else "")
        )

        # -- Hats --
        if status.hats:
            self.hats_var.set(", ".join(str(list(h)) for h in status.hats))
        else:
            self.hats_var.set("无")

        # -- Status bar --
        if status.last_error:
            self.status_bar_label.configure(
                text=f"错误: {status.last_error}", style="Error.TLabel",
            )
            self._set_indicator(self.status_bar_indicator, "bad")
        elif status.heave_conflict:
            self.status_bar_label.configure(
                text="A/Y 同时按下：heave 已强制归零",
                style="Ready.TLabel",
            )
            self._set_indicator(self.status_bar_indicator, "warn")
        else:
            self.status_bar_label.configure(text="就绪", style="Ready.TLabel")
            self._set_indicator(self.status_bar_indicator, "good")

        self.after(100, self._refresh)

    def _close(self) -> None:
        self.control_var.set(False)
        self.worker.close()
        self.destroy()


def main() -> None:
    App().mainloop()


if __name__ == "__main__":
    main()
