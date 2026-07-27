import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  normalizeStatus,
  type RobotStatus,
} from './status';
import {
  LOCKED_CAPABILITIES,
  normalizeCapabilities,
  type Capabilities,
} from './capabilities';
import { acceptTelemetryMessage, createTelemetryCursor } from './wsProtocol';
import {
  apiErrorMessage,
  backendReady,
  linkReady as isLinkReady,
  pwmControlAvailability,
  readinessView,
  sensorsFreshnessMessage,
  shouldClearLocalRequest,
} from './uiState';
import './App.css';

type NoticeKind = 'error' | 'warning' | 'info';

interface Notice {
  kind: NoticeKind;
  text: string;
}

type MotionAxis = 'surge' | 'sway' | 'heave' | 'roll' | 'pitch' | 'yaw';

interface MotionTuningValues {
  axis_gain: number[];
  axis_max_output: number[];
  global_multiplier: number;
  pwm_slew_rate_us_per_s: number;
  command_timeout_ms: number;
}

interface MotionTuningSnapshot {
  desired: MotionTuningValues;
  confirmed: MotionTuningValues | null;
  synced: boolean;
  sync_state: string;
  sync_error: string | null;
}

const DEFAULT_MOTION_TUNING: MotionTuningValues = {
  axis_gain: [1, 1, 1, 1, 1, 1],
  axis_max_output: [0.2, 0.2, 0.2, 0.1, 0.1, 0.1],
  global_multiplier: 1,
  pwm_slew_rate_us_per_s: 1000,
  command_timeout_ms: 500,
};

const MOTION_AXES: Array<{
  key: MotionAxis;
  name: string;
  positive: string;
  negative: string;
}> = [
  { key: 'surge', name: '纵向', positive: '前进', negative: '后退' },
  { key: 'sway', name: '横向', positive: '右移', negative: '左移' },
  { key: 'heave', name: '垂向', positive: '下潜', negative: '上浮' },
  { key: 'roll', name: '横滚', positive: '横滚 +', negative: '横滚 -' },
  { key: 'pitch', name: '俯仰', positive: '俯仰 +', negative: '俯仰 -' },
  { key: 'yaw', name: '偏航', positive: '右转', negative: '左转' },
];

function tuningValues(raw: unknown): MotionTuningValues | null {
  if (!raw || typeof raw !== 'object' || Array.isArray(raw)) return null;
  const value = raw as Record<string, unknown>;
  if (
    !Array.isArray(value.axis_gain)
    || !Array.isArray(value.axis_max_output)
    || value.axis_gain.length !== 6
    || value.axis_max_output.length !== 6
  ) return null;
  const gains = value.axis_gain.map(Number);
  const limits = value.axis_max_output.map(Number);
  const globalMultiplier = Number(value.global_multiplier);
  const slew = Number(value.pwm_slew_rate_us_per_s);
  const timeout = Number(value.command_timeout_ms);
  if (
    [...gains, ...limits, globalMultiplier, slew, timeout]
      .some(item => !Number.isFinite(item))
  ) return null;
  return {
    axis_gain: gains,
    axis_max_output: limits,
    global_multiplier: globalMultiplier,
    pwm_slew_rate_us_per_s: Math.round(slew),
    command_timeout_ms: Math.round(timeout),
  };
}

function normalizeTuningSnapshot(raw: unknown): MotionTuningSnapshot {
  if (!raw || typeof raw !== 'object' || Array.isArray(raw)) {
    throw new Error('运动参数响应格式无效');
  }
  const value = raw as Record<string, unknown>;
  const desired = tuningValues(value.desired);
  const confirmed = value.confirmed === null ? null : tuningValues(value.confirmed);
  if (!desired || (value.confirmed !== null && !confirmed)) {
    throw new Error('运动参数响应缺少六轴数据');
  }
  return {
    desired,
    confirmed,
    synced: value.synced === true,
    sync_state: typeof value.sync_state === 'string' ? value.sync_state : 'unknown',
    sync_error: typeof value.sync_error === 'string' ? value.sync_error : null,
  };
}

function useWebSocket() {
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [connected, setConnected] = useState(false);
  const [events, setEvents] = useState<string[]>([]);
  const [telemetryVersion, setTelemetryVersion] = useState(0);
  const [connectionVersion, setConnectionVersion] = useState(0);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimer = useRef<number | null>(null);
  const connectTimeout = useRef<number | null>(null);
  const intentionalClose = useRef(false);
  const connectionGeneration = useRef(0);
  const lastSession = useRef<string | null>(null);

  const addEvent = useCallback((msg: string) => {
    const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    setEvents(prev => [...prev.slice(-200), `[${ts}] ${msg}`]);
  }, []);

  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN || wsRef.current?.readyState === WebSocket.CONNECTING) {
      return;
    }

    intentionalClose.current = false;
    if (reconnectTimer.current) {
      clearTimeout(reconnectTimer.current);
      reconnectTimer.current = null;
    }

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const ws = new WebSocket(`${protocol}//${window.location.host}/ws/telemetry`);
    const generation = ++connectionGeneration.current;
    let cursor = createTelemetryCursor();
    wsRef.current = ws;
    const isCurrentConnection = () => (
      wsRef.current === ws && connectionGeneration.current === generation
    );

    ws.onopen = () => {
      if (!isCurrentConnection()) return;
      if (connectTimeout.current) {
        clearTimeout(connectTimeout.current);
        connectTimeout.current = null;
      }
      setConnected(true);
      setConnectionVersion(version => version + 1);
      addEvent('遥测连接已建立');
    };

    ws.onmessage = event => {
      if (!isCurrentConnection()) return;
      try {
        const decision = acceptTelemetryMessage(JSON.parse(event.data), cursor);
        if (!decision.accepted) return;
        cursor = decision.cursor;
        if (cursor.sessionId && lastSession.current && cursor.sessionId !== lastSession.current) {
          addEvent('检测到后端会话已重启，已重置遥测序列');
        }
        if (cursor.sessionId) lastSession.current = cursor.sessionId;
        setStatus(normalizeStatus(decision.payload));
        setTelemetryVersion(version => version + 1);
      } catch {
        addEvent('收到无法解析的遥测数据');
      }
    };

    ws.onclose = () => {
      if (!isCurrentConnection()) return;
      if (connectTimeout.current) {
        clearTimeout(connectTimeout.current);
        connectTimeout.current = null;
      }
      wsRef.current = null;
      connectionGeneration.current += 1;
      setConnected(false);
      setStatus(prev => prev ? { ...prev, status_stale: true, sensors_stale: true } : prev);

      if (intentionalClose.current) return;
      addEvent('遥测连接断开，正在重连');
      if (!reconnectTimer.current) {
        reconnectTimer.current = window.setTimeout(() => {
          reconnectTimer.current = null;
          connect();
        }, 2000);
      }
    };

    ws.onerror = () => {
      if (!isCurrentConnection()) return;
      ws.close();
    };

    connectTimeout.current = window.setTimeout(() => {
      if (isCurrentConnection() && ws.readyState === WebSocket.CONNECTING) {
        addEvent('遥测连接超时，正在重连');
        ws.close();
      }
    }, 5000);
  }, [addEvent]);

  useEffect(() => {
    connect();
    return () => {
      intentionalClose.current = true;
      connectionGeneration.current += 1;
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
      if (connectTimeout.current) {
        clearTimeout(connectTimeout.current);
        connectTimeout.current = null;
      }
      const ws = wsRef.current;
      wsRef.current = null;
      if (ws) {
        ws.onclose = null;
        ws.close();
      }
    };
  }, [connect]);

  return {
    status,
    connected,
    events,
    addEvent,
    telemetryVersion,
    connectionVersion,
  };
}

async function apiPost(path: string, body?: unknown, signal?: AbortSignal) {
  const controller = new AbortController();
  let timedOut = false;
  const abortFromCaller = () => controller.abort();
  if (signal?.aborted) abortFromCaller();
  else signal?.addEventListener('abort', abortFromCaller, { once: true });
  const timeout = window.setTimeout(() => {
    timedOut = true;
    controller.abort();
  }, 6000);

  try {
    const res = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: controller.signal,
    });

    if (!res.ok) {
      const err = await res.json().catch(() => ({ detail: res.statusText }));
      throw new Error(apiErrorMessage(err.detail, res.statusText || '请求失败'));
    }
    return res.json().catch(() => ({}));
  } catch (error) {
    if (timedOut) throw new Error('请求超时，未确认 STM32 是否执行');
    throw error;
  } finally {
    clearTimeout(timeout);
    signal?.removeEventListener('abort', abortFromCaller);
  }
}

async function apiGet(path: string, signal?: AbortSignal) {
  const controller = new AbortController();
  let timedOut = false;
  const abortFromCaller = () => controller.abort();
  if (signal?.aborted) abortFromCaller();
  else signal?.addEventListener('abort', abortFromCaller, { once: true });
  const timeout = window.setTimeout(() => {
    timedOut = true;
    controller.abort();
  }, 6000);
  try {
    const response = await fetch(path, { signal: controller.signal });
    if (!response.ok) {
      const error = await response.json().catch(() => ({ detail: response.statusText }));
      throw new Error(apiErrorMessage(error.detail, response.statusText || '请求失败'));
    }
    return response.json();
  } catch (error) {
    if (timedOut) throw new Error('请求超时');
    throw error;
  } finally {
    clearTimeout(timeout);
    signal?.removeEventListener('abort', abortFromCaller);
  }
}

function useCapabilities(reloadKey: number) {
  const [capabilities, setCapabilities] = useState<Capabilities>(LOCKED_CAPABILITIES);
  const [ready, setReady] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const controller = new AbortController();
    let disposed = false;
    let timedOut = false;
    const timeout = window.setTimeout(() => {
      timedOut = true;
      controller.abort();
    }, 5000);
    const load = async () => {
      try {
        const response = await fetch('/api/capabilities', { signal: controller.signal });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const parsed = normalizeCapabilities(await response.json());
        setCapabilities(parsed);
        setReady(true);
        setError(null);
      } catch (e: unknown) {
        if (disposed) return;
        setCapabilities(LOCKED_CAPABILITIES);
        setReady(false);
        setError(timedOut
          ? '能力配置请求超时'
          : e instanceof Error ? e.message : '未知配置错误');
      }
    };
    void load();
    return () => {
      disposed = true;
      clearTimeout(timeout);
      controller.abort();
    };
  }, [reloadKey]);

  return { capabilities, ready, error };
}

function formatNumber(value: number, precision: number, unit = '') {
  if (!Number.isFinite(value)) return '--';
  return `${value.toFixed(precision)}${unit}`;
}

function formatPwm(value: number | undefined) {
  return Number.isFinite(value) ? `${value}us` : '--';
}

function formatAge(value: number | null) {
  if (!Number.isFinite(value)) return '从未收到';
  if ((value as number) < 1000) return `${value}ms`;
  return `${((value as number) / 1000).toFixed(1)}s`;
}

function requestStateLabel(value: RobotStatus['request_state']) {
  const labels: Record<RobotStatus['request_state'], string> = {
    idle: '空闲',
    pending: '等待确认',
    confirmed: '状态已确认',
    rejected: '已拒绝',
    timeout: '确认超时',
    unknown: '未知',
  };
  return labels[value];
}

function formatTime(seconds: number) {
  if (!seconds) return '从未';
  return new Date(seconds * 1000).toLocaleTimeString('zh-CN', { hour12: false });
}

function modeLabel(mode?: string) {
  if (mode === 'SIMULATION') return '仿真';
  if (mode === 'REAL HARDWARE') return '真实硬件';
  return mode || '未知';
}

function stateTone(state?: number) {
  if (state === 5 || state === 6) return 'danger';
  if (state === 4) return 'warning';
  if (state === 3) return 'manual';
  if (state === 1 || state === 2) return 'ok';
  return 'muted';
}

function StatusBadge({ label, tone = 'muted' }: { label: string; tone?: string }) {
  return <span className={`status-badge status-badge--${tone}`}>{label}</span>;
}

function Panel({
  title,
  eyebrow,
  children,
  className = '',
}: {
  title: string;
  eyebrow?: string;
  children: React.ReactNode;
  className?: string;
}) {
  return (
    <section className={`panel ${className}`}>
      <div className="panel__head">
        <div>
          {eyebrow && <div className="panel__eyebrow">{eyebrow}</div>}
          <h2>{title}</h2>
        </div>
      </div>
      {children}
    </section>
  );
}

function Metric({
  label,
  value,
  hint,
  tone = 'normal',
}: {
  label: string;
  value: string;
  hint?: string;
  tone?: 'normal' | 'good' | 'warn' | 'danger' | 'muted';
}) {
  return (
    <div className={`metric metric--${tone}`}>
      <div className="metric__label">{label}</div>
      <div className="metric__value">{value}</div>
      {hint && <div className="metric__hint">{hint}</div>}
    </div>
  );
}

function Header({
  status,
  connected,
  backendReady,
  linkReady,
  motionInhibited,
  pendingAction,
  estopPending,
  onArm,
  onDisarm,
  onResetEstop,
  onEstop,
}: {
  status: RobotStatus | null;
  connected: boolean;
  backendReady: boolean;
  linkReady: boolean;
  motionInhibited: boolean;
  pendingAction: string | null;
  estopPending: boolean;
  onArm: () => void;
  onDisarm: () => void;
  onResetEstop: () => void;
  onEstop: () => void;
}) {
  const busy = Boolean(pendingAction);
  const canArm = Boolean(
    status
    && status.safety_state === 0
    && status.estop_locked === false
    && status.control_mode === 'IDLE',
  );
  const canDisarm = Boolean(status && (
    motionInhibited
    || (status.safety_state !== 0 && status.safety_state !== 5)
  ));
  const canReset = Boolean(status && status.safety_state === 5);

  return (
    <header className="topbar">
      <div className="topbar__identity">
        <div className="brand-mark">1132</div>
        <div>
          <h1>水下机器人调试控制台</h1>
          <p>Orange Pi 与 STM32 状态、传感器和 PWM 安全测试</p>
        </div>
      </div>

      <div className="topbar__status">
        <StatusBadge label={connected ? 'WebSocket 已连接' : 'WebSocket 断开'} tone={connected ? 'ok' : 'danger'} />
        <StatusBadge label={status?.safety_state_label ?? '等待数据'} tone={stateTone(status?.safety_state)} />
        <StatusBadge
          label={`控制模式 ${status?.control_mode ?? 'UNKNOWN'}`}
          tone={status?.control_mode === 'IDLE' ? 'muted' : 'manual'}
        />
      </div>

      <div className="topbar__actions">
        {canArm && (
          <button className="btn btn--primary" disabled={!backendReady || motionInhibited || busy} onClick={onArm}>
            解锁
          </button>
        )}
        {canDisarm && (
          <button className="btn btn--secondary" disabled={!linkReady || busy} onClick={onDisarm}>
            上锁并回中
          </button>
        )}
        {canReset && (
          <button className="btn btn--warning" disabled={!linkReady || busy} onClick={onResetEstop}>
            解除急停
          </button>
        )}
        <button className="btn btn--estop" disabled={estopPending} onClick={onEstop}>
          {estopPending ? '急停发送中' : '急停'}
        </button>
      </div>
    </header>
  );
}

function SystemPanel({ status, connected }: { status: RobotStatus | null; connected: boolean }) {
  if (!status) {
    return (
      <Panel title="系统状态" eyebrow="SYSTEM">
        <div className="empty-state">正在等待遥测数据</div>
      </Panel>
    );
  }

  const serialValue = connected ? (status.serial_connected ? '正常' : '断开') : '未知（前端离线）';
  const stm32Value = connected ? (status.stm32_online ? '在线' : '离线') : '未知（前端离线）';
  const serialTone = connected ? (status.serial_connected ? 'good' : 'danger') : 'muted';
  const stm32Tone = connected ? (status.stm32_online ? 'good' : 'danger') : 'muted';

  return (
    <Panel title="系统状态" eyebrow="SYSTEM">
      <div className="system-state">
        <div>
          <div className="system-state__label">当前状态</div>
          <div className={`system-state__value system-state__value--${stateTone(status.safety_state)}`}>
            {status.safety_state_label}
          </div>
          <div className="system-state__raw">{status.safety_state_name}</div>
        </div>
        <StatusBadge label={modeLabel(status.mode)} tone={status.mode === 'SIMULATION' ? 'manual' : 'ok'} />
      </div>

      <div className="metric-grid metric-grid--two">
        <Metric label="前端连接" value={connected ? '正常' : '断开'} tone={connected ? 'good' : 'danger'} />
        <Metric label="串口链路" value={serialValue} tone={serialTone} />
        <Metric label="STM32" value={stm32Value} tone={stm32Tone} />
        <Metric
          label="状态数据"
          value={status.status_stale ? '已过期' : '新鲜'}
          hint={formatAge(status.status_age_ms)}
          tone={status.status_stale ? 'warn' : 'good'}
        />
        <Metric
          label="传感器数据"
          value={status.sensors_stale ? '已过期' : '新鲜'}
          hint={formatAge(status.sensors_age_ms)}
          tone={status.sensors_stale ? 'warn' : 'good'}
        />
        <Metric label="控制使能" value={status.control_enable ? '已开启' : '关闭'} tone={status.control_enable ? 'good' : 'muted'} />
        <Metric
          label="后端运动锁"
          value={status.backend_motion_inhibited ? '锁定' : '正常'}
          hint={status.backend_motion_inhibit_reason ?? undefined}
          tone={status.backend_motion_inhibited ? 'danger' : 'good'}
        />
        <Metric label="定深" value={status.float_enabled ? '开启' : '关闭'} tone={status.float_enabled ? 'good' : 'muted'} />
        <Metric label="角度闭环" value={status.angle_enabled ? '开启' : '关闭'} tone={status.angle_enabled ? 'good' : 'muted'} />
        <Metric label="六轴控制" value={status.body_control_enabled ? '开启' : '关闭'} tone={status.body_control_enabled ? 'good' : 'muted'} />
        <Metric
          label="运动参数"
          value={status.motion_tuning_synced ? '已同步' : '未同步'}
          hint={status.motion_tuning_sync_error ?? status.motion_tuning_sync_state}
          tone={status.motion_tuning_synced ? 'good' : 'warn'}
        />
        <Metric label="协议错误" value={String(status.error_count)} tone={status.error_count > 0 ? 'warn' : 'muted'} />
        <Metric label="心跳丢失" value={String(status.heartbeat_missed)} tone={status.heartbeat_missed > 0 ? 'warn' : 'muted'} />
      </div>
    </Panel>
  );
}

function SensorPanel({ status, connected }: { status: RobotStatus | null; connected: boolean }) {
  if (!status) {
    return (
      <Panel title="传感器" eyebrow="SENSORS">
        <div className="empty-state">暂无传感器数据</div>
      </Panel>
    );
  }

  const staleMessage = sensorsFreshnessMessage(connected, status);

  return (
    <Panel title="传感器" eyebrow="SENSORS">
      {staleMessage && <div className="inline-warning">{staleMessage}</div>}
      <div className="sensor-grid">
        <Metric label="深度" value={formatNumber(status.depth_m, 2, ' m')} />
        <Metric label="压力" value={formatNumber(status.pressure_mbar, 1, ' mbar')} />
        <Metric label="水温" value={formatNumber(status.water_temp_c, 1, ' °C')} />
        <Metric label="航向" value={formatNumber(status.yaw * 57.2958, 1, '°')} />
        <Metric label="俯仰" value={formatNumber(status.pitch * 57.2958, 1, '°')} />
        <Metric label="横滚" value={formatNumber(status.roll * 57.2958, 1, '°')} />
      </div>
      <div className="vector-strip">
        <span>加速度 {status.accel.map(v => formatNumber(v, 2)).join(' / ')}</span>
        <span>陀螺仪 {status.gyro.map(v => formatNumber(v, 2)).join(' / ')}</span>
        <span>磁力计 {status.mag.map(v => formatNumber(v, 1)).join(' / ')}</span>
      </div>
    </Panel>
  );
}

function ReadinessStrip({
  status,
  connected,
  capabilitiesReady,
}: {
  status: RobotStatus | null;
  connected: boolean;
  capabilitiesReady: boolean;
}) {
  const view = readinessView(connected, status, capabilitiesReady);
  return <div className={`readiness readiness--${view.tone}`}>{view.text}</div>;
}

function PwmPanel({
  status,
  connected,
  capabilities,
  capabilitiesReady,
  safetyLocked,
  telemetryVersion,
  onError,
  onEvent,
}: {
  status: RobotStatus | null;
  connected: boolean;
  capabilities: Capabilities;
  capabilitiesReady: boolean;
  safetyLocked: boolean;
  telemetryVersion: number;
  onError: (msg: string) => void;
  onEvent: (msg: string) => void;
}) {
  const [selectedChannel, setSelectedChannel] = useState(0);
  const [pwmValue, setPwmValue] = useState(capabilities.pwm.neutral_us);
  const [duration, setDuration] = useState(capabilities.pwm.default_timeout_ms);
  const [testing, setTesting] = useState(false);
  const [localRequest, setLocalRequest] = useState<{
    kind: 'test' | 'neutral';
    channel?: number;
    pwm?: number;
    acceptedAfterVersion: number;
  } | null>(null);
  const requestEpoch = useRef(0);
  const requestController = useRef<AbortController | null>(null);
  const requestInFlight = useRef(false);

  const motionReady = backendReady(connected, status);
  const availability = pwmControlAvailability({
    connected,
    status,
    capabilitiesReady,
    safetyLocked,
    requestPending: testing,
  });
  const canTest = (
    availability.canTest
    && capabilities.features.manual_pwm
    && status?.control_mode === 'MOTOR_TEST'
  );
  const canNeutral = availability.canNeutral;
  const channels = useMemo(
    () => Array.from({ length: capabilities.channel_count }, (_, index) => index),
    [capabilities.channel_count],
  );
  const durationOptions = useMemo(() => Array.from(new Set([
    capabilities.pwm.min_test_duration_ms,
    capabilities.pwm.default_timeout_ms,
    500,
    1000,
    capabilities.pwm.max_test_duration_ms,
  ])).filter(value => (
    value >= capabilities.pwm.min_test_duration_ms
    && value <= capabilities.pwm.max_test_duration_ms
  )).sort((a, b) => a - b), [capabilities]);

  useEffect(() => {
    setSelectedChannel(channel => Math.min(channel, capabilities.channel_count - 1));
    setPwmValue(value => Math.min(
      capabilities.pwm.max_test_us,
      Math.max(capabilities.pwm.min_test_us, value),
    ));
    setDuration(value => (
      value >= capabilities.pwm.min_test_duration_ms
      && value <= capabilities.pwm.max_test_duration_ms
        ? value
        : capabilities.pwm.default_timeout_ms
    ));
  }, [capabilities]);

  useEffect(() => {
    if (!shouldClearLocalRequest(connected, safetyLocked)) return;
    requestEpoch.current += 1;
    requestController.current?.abort();
    requestController.current = null;
    requestInFlight.current = false;
    setTesting(false);
    setLocalRequest(null);
  }, [connected, safetyLocked]);

  useEffect(() => () => {
    requestEpoch.current += 1;
    requestController.current?.abort();
    requestInFlight.current = false;
  }, []);

  useEffect(() => {
    if (!localRequest || !status || telemetryVersion <= localRequest.acceptedAfterVersion) return;
    if (status.request_state === 'rejected' || status.request_state === 'timeout') {
      onError(status.last_command_error
        ?? (status.request_state === 'timeout' ? 'PWM 请求确认超时' : 'PWM 请求被拒绝'));
      setLocalRequest(null);
      return;
    }
    if (status.status_stale) return;

    if (localRequest.kind === 'neutral') {
      const confirmedNeutral = channels.every(channel => (
        Number.isFinite(status.confirmed_pwm[channel])
        && status.confirmed_pwm[channel] === capabilities.pwm.neutral_us
      ));
      if (confirmedNeutral) {
        onEvent('STM32 状态已确认：全部 PWM 已回中');
        setLocalRequest(null);
      }
      return;
    }

    const channel = localRequest.channel;
    if (
      channel !== undefined
      && Number.isFinite(status.confirmed_pwm[channel])
      && status.confirmed_pwm[channel] === localRequest.pwm
    ) {
      onEvent(`STM32 状态已确认：CH${channel} 为 ${localRequest.pwm}us`);
      setLocalRequest(null);
      return;
    }

    const timedPulseReturnedNeutral = (
      status.request_state === 'idle'
      && channels.every(index => (
        Number.isFinite(status.confirmed_pwm[index])
        && status.confirmed_pwm[index] === capabilities.pwm.neutral_us
      ))
    );
    if (timedPulseReturnedNeutral) {
      onEvent('STM32 状态已确认：短脉冲测试已结束并自动回中');
      setLocalRequest(null);
    }
  }, [capabilities.pwm.neutral_us, channels, localRequest, onError, onEvent, status, telemetryVersion]);

  const manualHint = useMemo(() => {
    if (!status) return '等待遥测数据';
    if (!motionReady) return status?.backend_motion_inhibited
      ? '后端运动锁定，请先上锁回中'
      : '链路未就绪';
    if (!capabilitiesReady) return '安全能力配置不可用';
    if (!capabilities.features.manual_pwm) return '后端未启用手动 PWM 能力';
    if (safetyLocked || status.estop_locked) return '急停或全局安全锁已启用';
    if (
      status.control_mode !== 'IDLE'
      && status.control_mode !== 'MOTOR_TEST'
    ) return `${status.control_mode} 正在占用运动控制`;
    if (status.safety_state === 0) return '请先解锁';
    if (status.safety_state === 1) return '请进入手动测试';
    if (status.safety_state === 3) return '可进行单通道短脉冲测试';
    if (status.safety_state === 5) return '急停锁定';
    return '当前状态不允许手动 PWM';
  }, [motionReady, capabilities, capabilitiesReady, safetyLocked, status]);

  const startRequest = () => {
    if (requestInFlight.current) return null;
    requestInFlight.current = true;
    const controller = new AbortController();
    const epoch = ++requestEpoch.current;
    requestController.current = controller;
    setTesting(true);
    return { controller, epoch };
  };

  const finishRequest = (epoch: number) => {
    if (requestEpoch.current !== epoch) return;
    requestController.current = null;
    requestInFlight.current = false;
    setTesting(false);
  };

  const doNeutral = async () => {
    if (!canNeutral) return;
    const request = startRequest();
    if (!request) return;
    const { controller, epoch } = request;
    try {
      await apiPost('/api/pwm/neutral', undefined, controller.signal);
      if (requestEpoch.current !== epoch) return;
      setLocalRequest({ kind: 'neutral', acceptedAfterVersion: telemetryVersion });
      onEvent('回中请求已接受，等待 STM32 状态确认');
    } catch (e: unknown) {
      if (controller.signal.aborted) return;
      onError(`回中失败：${e instanceof Error ? e.message : '未知错误'}`);
    } finally {
      finishRequest(epoch);
    }
  };

  const doTest = async () => {
    if (!canTest) return;
    const request = startRequest();
    if (!request) return;
    const { controller, epoch } = request;
    try {
      await apiPost('/api/pwm/test', {
        channel: selectedChannel,
        pwm_us: pwmValue,
        duration_ms: duration,
      }, controller.signal);
      if (requestEpoch.current !== epoch) return;
      setLocalRequest({
        kind: 'test',
        channel: selectedChannel,
        pwm: pwmValue,
        acceptedAfterVersion: telemetryVersion,
      });
      onEvent(`CH${selectedChannel} ${pwmValue}us/${duration}ms 请求已接受，等待状态确认`);
    } catch (e: unknown) {
      if (controller.signal.aborted) return;
      onError(`PWM 测试失败：${e instanceof Error ? e.message : '未知错误'}`);
    } finally {
      finishRequest(epoch);
    }
  };

  const runModeCommand = async (path: string, acceptedMessage: string, failurePrefix: string) => {
    if (testing || !motionReady || !capabilitiesReady || safetyLocked) return;
    const request = startRequest();
    if (!request) return;
    const { controller, epoch } = request;
    try {
      await apiPost(path, undefined, controller.signal);
      if (requestEpoch.current !== epoch) return;
      onEvent(acceptedMessage);
    } catch (e: unknown) {
      if (controller.signal.aborted) return;
      onError(`${failurePrefix}：${e instanceof Error ? e.message : '未知错误'}`);
    } finally {
      finishRequest(epoch);
    }
  };

  const enterManual = () => runModeCommand('/api/enter-manual', '进入手动测试请求已接受', '进入手动测试失败');
  const exitManual = () => runModeCommand('/api/exit-manual', '退出手动测试请求已接受', '退出手动测试失败');

  const switchChannel = (channel: number) => {
    if (!canTest || channel === selectedChannel) return;
    const confirmedNeutral = channels.every(index => (
      Number.isFinite(status?.confirmed_pwm[index])
      && status?.confirmed_pwm[index] === capabilities.pwm.neutral_us
    ));
    if (!confirmedNeutral) {
      onError('切换通道前请先请求全部回中，并等待 STM32 状态确认');
      return;
    }
    setSelectedChannel(channel);
  };

  const setPwmClamped = (value: number) => {
    if (!Number.isFinite(value)) return;
    setPwmValue(Math.min(
      capabilities.pwm.max_test_us,
      Math.max(capabilities.pwm.min_test_us, Math.round(value)),
    ));
  };

  if (!status) {
    return (
      <Panel title="PWM 手动测试" eyebrow="PWM">
        <div className="empty-state">暂无 PWM 状态</div>
      </Panel>
    );
  }

  return (
    <Panel title="PWM 手动测试" eyebrow="PWM" className="panel--wide">
      <div className="control-summary">
        <div>
          <div className="control-summary__label">测试状态</div>
          <div className={`control-summary__value ${canTest ? 'is-ready' : ''}`}>{manualHint}</div>
        </div>
        <div className="control-summary__actions">
          {status.safety_state === 1 && (
            <button
              className="btn btn--primary"
              disabled={
                !motionReady
                || !capabilitiesReady
                || safetyLocked
                || testing
                || status.control_mode !== 'IDLE'
              }
              onClick={enterManual}
            >
              进入手动测试
            </button>
          )}
          {status.safety_state === 3 && (
            <button className="btn btn--secondary" disabled={!motionReady || !capabilitiesReady || safetyLocked || testing} onClick={exitManual}>
              退出手动测试
            </button>
          )}
        </div>
      </div>

      <div className="channel-grid">
        {channels.map(channel => {
          const pwm = status.confirmed_pwm[channel];
          const active = Number.isFinite(pwm) && Math.abs(pwm - capabilities.pwm.neutral_us) >= 1;
          const selected = channel === selectedChannel;
          return (
            <button
              key={channel}
              className={`channel-button ${selected ? 'is-selected' : ''} ${active ? 'is-active' : ''}`}
              disabled={!canTest}
              onClick={() => switchChannel(channel)}
            >
              <span>CH{channel}</span>
              <strong>{formatPwm(pwm)}</strong>
              <small>{!Number.isFinite(pwm) ? '确认未知' : status.status_stale ? '过期缓存' : active ? '确认输出' : '确认中位'}</small>
            </button>
          );
        })}
      </div>

      <div className="pwm-controls">
        <label className="slider-block">
          <span>PWM 脉宽：{pwmValue}us</span>
          <div className="pwm-value-editor">
            <button type="button" disabled={!canTest} onClick={() => setPwmClamped(pwmValue - 10)}>-10</button>
            <button type="button" disabled={!canTest} onClick={() => setPwmClamped(pwmValue - 1)}>-1</button>
            <input
              type="number"
              min={capabilities.pwm.min_test_us}
              max={capabilities.pwm.max_test_us}
              step={1}
              value={pwmValue}
              disabled={!canTest}
              aria-label="PWM 脉宽，单位微秒"
              onChange={event => setPwmClamped(Number(event.target.value))}
            />
            <button type="button" disabled={!canTest} onClick={() => setPwmClamped(pwmValue + 1)}>+1</button>
            <button type="button" disabled={!canTest} onClick={() => setPwmClamped(pwmValue + 10)}>+10</button>
          </div>
          <input
            type="range"
            min={capabilities.pwm.min_test_us}
            max={capabilities.pwm.max_test_us}
            step={1}
            value={pwmValue}
            disabled={!canTest}
            onChange={event => setPwmClamped(Number(event.target.value))}
          />
          <div className="slider-scale">
            <span>{capabilities.pwm.min_test_us} 反向</span>
            <span>{capabilities.pwm.neutral_us} 停止</span>
            <span>{capabilities.pwm.max_test_us} 正向</span>
          </div>
        </label>

        <div className="duration-group">
          <span>持续时间</span>
          <div>
            {durationOptions.map(item => (
              <button
                key={item}
                className={`segmented ${duration === item ? 'is-active' : ''}`}
                disabled={!canTest}
                onClick={() => setDuration(item)}
              >
                {item}ms
              </button>
            ))}
          </div>
        </div>

        <div className="test-actions">
          <button className="btn btn--primary btn--large" disabled={!canTest} onClick={doTest}>
            {testing ? '测试中' : `输出 CH${selectedChannel}`}
          </button>
          <button className="btn btn--secondary btn--large" disabled={!canNeutral} onClick={doNeutral}>
            全部回中
          </button>
        </div>
      </div>

      {localRequest && (
        <div className="inline-info">
          请求已被后端接受，只有收到新的 STM32 状态报告后才会标记为已执行。
        </div>
      )}

      {status.last_command_error && (
        <div className="inline-error">最近命令错误：{status.last_command_error}</div>
      )}

      {testing && (
        <div className="active-warning">
          控制请求发送中。尚未收到响应，请勿重复操作。
        </div>
      )}
    </Panel>
  );
}

function MotionTuningPanel({
  status,
  connected,
  capabilities,
  capabilitiesReady,
  safetyLocked,
  connectionVersion,
  onError,
  onEvent,
}: {
  status: RobotStatus | null;
  connected: boolean;
  capabilities: Capabilities;
  capabilitiesReady: boolean;
  safetyLocked: boolean;
  connectionVersion: number;
  onError: (msg: string) => void;
  onEvent: (msg: string) => void;
}) {
  const [tuning, setTuning] = useState<MotionTuningValues>(DEFAULT_MOTION_TUNING);
  const [snapshot, setSnapshot] = useState<MotionTuningSnapshot | null>(null);
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [modeBusy, setModeBusy] = useState(false);
  const [heldButton, setHeldButton] = useState<string | null>(null);
  const heldRef = useRef<{ axis: MotionAxis; value: number; epoch: number } | null>(null);
  const holdTimerRef = useRef<number | null>(null);
  const holdEpochRef = useRef(0);
  const commandInFlightRef = useRef(false);
  const commandPromiseRef = useRef<Promise<void> | null>(null);
  const modeActiveRef = useRef(false);

  const applySnapshot = useCallback((value: MotionTuningSnapshot) => {
    setSnapshot(value);
    setTuning({
      ...value.desired,
      axis_gain: [...value.desired.axis_gain],
      axis_max_output: [...value.desired.axis_max_output],
    });
  }, []);

  const reloadTuning = useCallback(async (announce = true) => {
    if (!connected || !capabilitiesReady || !capabilities.features.motion_tuning) return;
    setLoading(true);
    try {
      const value = normalizeTuningSnapshot(await apiGet('/api/motion/tuning'));
      applySnapshot(value);
      if (announce) onEvent('运动参数已重新加载');
    } catch (error) {
      onError(`加载运动参数失败：${error instanceof Error ? error.message : '未知错误'}`);
    } finally {
      setLoading(false);
    }
  }, [
    applySnapshot,
    capabilities.features.motion_tuning,
    capabilitiesReady,
    connected,
    onError,
    onEvent,
  ]);

  useEffect(() => {
    if (!connected || !capabilitiesReady || !capabilities.features.motion_tuning) return;
    const controller = new AbortController();
    setLoading(true);
    void apiGet('/api/motion/tuning', controller.signal)
      .then(value => applySnapshot(normalizeTuningSnapshot(value)))
      .catch(error => {
        if (!controller.signal.aborted) {
          onError(`加载运动参数失败：${error instanceof Error ? error.message : '未知错误'}`);
        }
      })
      .finally(() => {
        if (!controller.signal.aborted) setLoading(false);
      });
    return () => controller.abort();
  }, [
    applySnapshot,
    capabilities.features.motion_tuning,
    capabilitiesReady,
    connected,
    connectionVersion,
    onError,
  ]);

  const clamp = (value: number, min: number, max: number) => (
    Math.min(max, Math.max(min, value))
  );

  const setAxisValue = (
    field: 'axis_gain' | 'axis_max_output',
    index: number,
    value: number,
  ) => {
    if (!Number.isFinite(value)) return;
    const limits = capabilities.motion_tuning;
    const min = field === 'axis_gain' ? limits.gain_min : limits.axis_max_output_min;
    const max = field === 'axis_gain' ? limits.gain_max : limits.axis_max_output_max;
    setTuning(current => {
      const next = [...current[field]];
      next[index] = clamp(value, min, max);
      return { ...current, [field]: next };
    });
  };

  const stopped = Boolean(status && (
    status.safety_state === 0 || status.safety_state === 1
  ));
  const tuningReady = Boolean(
    connected
    && capabilitiesReady
    && capabilities.features.motion_tuning
    && status
    && status.stm32_online
    && !status.status_stale,
  );
  const canSave = tuningReady && stopped && !safetyLocked && !loading && !saving && !modeBusy;
  const modeActive = Boolean(
    status?.safety_state === 2
    && status.body_control_enabled
    && status.control_mode === 'WEB_MOTION',
  );
  modeActiveRef.current = modeActive;
  const canEnable = Boolean(
    tuningReady
    && status?.safety_state === 1
    && status?.control_mode === 'IDLE'
    && !safetyLocked
    && !modeBusy,
  );
  const canDisable = Boolean(tuningReady && modeActive && !modeBusy);
  const holdAllowed = Boolean(
    tuningReady
    && modeActive
    && status?.motion_tuning_synced
    && !safetyLocked
    && !modeBusy,
  );

  const saveTuning = async () => {
    if (!canSave) return;
    setSaving(true);
    try {
      const value = normalizeTuningSnapshot(
        await apiPost('/api/motion/tuning', tuning),
      );
      applySnapshot(value);
      onEvent('运动参数已保存到 Orange Pi，并由 STM32 回读确认');
    } catch (error) {
      onError(`保存运动参数失败：${error instanceof Error ? error.message : '未知错误'}`);
    } finally {
      setSaving(false);
    }
  };

  const clearLocalHold = useCallback(() => {
    holdEpochRef.current += 1;
    heldRef.current = null;
    if (holdTimerRef.current !== null) {
      clearInterval(holdTimerRef.current);
      holdTimerRef.current = null;
    }
    setHeldButton(null);
  }, []);

  const releaseHold = useCallback((announce = false) => {
    if (!heldRef.current) return;
    const pendingCommand = commandPromiseRef.current;
    clearLocalHold();
    const sendStop = (reportSuccess: boolean) => apiPost('/api/motion/stop')
      .then(() => {
        if (reportSuccess) onEvent('六轴按键已松开，零指令已发送');
      })
      .catch(error => {
        onError(`六轴停止失败：${error instanceof Error ? error.message : '未知错误'}`);
      });
    void sendStop(announce);
    if (pendingCommand) {
      void pendingCommand.finally(() => sendStop(false)).catch(() => undefined);
    }
  }, [clearLocalHold, onError, onEvent]);

  const sendHeldCommand = useCallback(async (epoch: number) => {
    const held = heldRef.current;
    if (!held || held.epoch !== epoch || commandInFlightRef.current) return;
    commandInFlightRef.current = true;
    const command: Record<MotionAxis, number> = {
      surge: 0,
      sway: 0,
      heave: 0,
      roll: 0,
      pitch: 0,
      yaw: 0,
    };
    command[held.axis] = held.value;
    const operation = apiPost('/api/motion/command', command)
      .then(() => undefined)
      .catch(error => {
        if (heldRef.current?.epoch === epoch) {
          clearLocalHold();
          onError(`六轴控制命令失败：${error instanceof Error ? error.message : '未知错误'}`);
          void apiPost('/api/motion/stop').catch(() => undefined);
        }
      });
    commandPromiseRef.current = operation;
    try {
      await operation;
    } finally {
      if (commandPromiseRef.current === operation) {
        commandPromiseRef.current = null;
      }
      commandInFlightRef.current = false;
    }
  }, [clearLocalHold, onError]);

  const startHold = (
    event: React.PointerEvent<HTMLButtonElement>,
    axis: MotionAxis,
    value: number,
  ) => {
    if (!holdAllowed || heldRef.current) return;
    event.preventDefault();
    event.currentTarget.setPointerCapture?.(event.pointerId);
    const epoch = ++holdEpochRef.current;
    heldRef.current = { axis, value, epoch };
    setHeldButton(`${axis}:${value}`);
    void sendHeldCommand(epoch);
    holdTimerRef.current = window.setInterval(() => {
      void sendHeldCommand(epoch);
    }, 100);
  };

  useEffect(() => {
    const stop = () => releaseHold(false);
    const visibility = () => {
      if (document.hidden) stop();
    };
    window.addEventListener('pointerup', stop);
    window.addEventListener('blur', stop);
    document.addEventListener('visibilitychange', visibility);
    return () => {
      window.removeEventListener('pointerup', stop);
      window.removeEventListener('blur', stop);
      document.removeEventListener('visibilitychange', visibility);
    };
  }, [releaseHold]);

  useEffect(() => {
    if (!holdAllowed && heldRef.current) releaseHold(false);
  }, [holdAllowed, releaseHold]);

  useEffect(() => () => {
    const wasHeld = Boolean(heldRef.current);
    const pendingCommand = commandPromiseRef.current;
    clearLocalHold();
    if (modeActiveRef.current) {
      void apiPost('/api/motion/disable').catch(() => undefined);
      if (pendingCommand) {
        void pendingCommand.finally(
          () => apiPost('/api/motion/disable'),
        ).catch(() => undefined);
      }
    } else if (wasHeld) {
      void apiPost('/api/motion/stop').catch(() => undefined);
      if (pendingCommand) {
        void pendingCommand.finally(
          () => apiPost('/api/motion/stop'),
        ).catch(() => undefined);
      }
    }
  }, [clearLocalHold]);

  const runModeCommand = async (path: string, success: string) => {
    if (modeBusy) return;
    setModeBusy(true);
    try {
      await apiPost(path);
      onEvent(success);
      await reloadTuning(false);
    } catch (error) {
      onError(`${success}失败：${error instanceof Error ? error.message : '未知错误'}`);
    } finally {
      setModeBusy(false);
    }
  };

  const disableMotion = () => {
    releaseHold(false);
    void runModeCommand('/api/motion/disable', '六轴控制已退出并硬回中');
  };

  const limits = capabilities.motion_tuning;
  const syncConfirmed = status?.motion_tuning_synced ?? snapshot?.synced ?? false;
  const syncError = status?.motion_tuning_sync_error ?? snapshot?.sync_error;

  return (
    <Panel title="运动调参" eyebrow="6-DOF MOTION" className="panel--wide">
      {!capabilities.features.motion_tuning && (
        <div className="inline-error">后端未启用运动调参能力。</div>
      )}

      <div className="motion-toolbar">
        <div>
          <div className="control-summary__label">参数与模式状态</div>
          <div className={`control-summary__value ${syncConfirmed ? 'is-ready' : ''}`}>
            {syncConfirmed ? '参数已由 STM32 回读确认' : syncError ?? '等待参数同步'}
          </div>
        </div>
        <div className="control-summary__actions">
          <button className="btn btn--secondary" disabled={!tuningReady || loading} onClick={() => void reloadTuning()}>
            {loading ? '加载中' : '重新加载'}
          </button>
          <button className="btn btn--primary" disabled={!canSave} onClick={() => void saveTuning()}>
            {saving ? '保存中' : '保存并同步'}
          </button>
          {status?.safety_state === 1 && (
            <button
              className="btn btn--primary"
              disabled={!canEnable}
              onClick={() => void runModeCommand('/api/motion/enable', '六轴控制已启用')}
            >
              {modeBusy ? '处理中' : '进入六轴控制'}
            </button>
          )}
          {modeActive && (
            <button className="btn btn--warning" disabled={!canDisable} onClick={disableMotion}>
              停止并退出
            </button>
          )}
        </div>
      </div>

      {!stopped && (
        <div className="inline-warning">
          当前处于运行状态，参数输入已锁定。先松开按键并点击“停止并退出”，再保存参数。
        </div>
      )}

      <div className="tuning-table">
        <div className="tuning-table__head">
          <span>轴</span>
          <span>轴增益</span>
          <span>最大输出</span>
        </div>
        {MOTION_AXES.map((axis, index) => (
          <div className="tuning-table__row" key={axis.key}>
            <div>
              <strong>{axis.name}</strong>
              <small>{axis.key}</small>
            </div>
            <label>
              <input
                type="number"
                min={limits.gain_min}
                max={limits.gain_max}
                step={0.01}
                value={tuning.axis_gain[index]}
                disabled={!stopped || !tuningReady}
                onChange={event => setAxisValue('axis_gain', index, Number(event.target.value))}
              />
              <span>倍</span>
            </label>
            <label>
              <input
                type="number"
                min={limits.axis_max_output_min * 100}
                max={limits.axis_max_output_max * 100}
                step={1}
                value={Math.round(tuning.axis_max_output[index] * 100)}
                disabled={!stopped || !tuningReady}
                onChange={event => setAxisValue(
                  'axis_max_output',
                  index,
                  Number(event.target.value) / 100,
                )}
              />
              <span>%</span>
            </label>
          </div>
        ))}
      </div>

      <div className="global-tuning-grid">
        <label>
          <span>全局倍率</span>
          <div>
            <input
              type="range"
              min={limits.global_multiplier_min * 100}
              max={limits.global_multiplier_max * 100}
              step={1}
              value={Math.round(tuning.global_multiplier * 100)}
              disabled={!stopped || !tuningReady}
              onChange={event => setTuning(current => ({
                ...current,
                global_multiplier: clamp(
                  Number(event.target.value) / 100,
                  limits.global_multiplier_min,
                  limits.global_multiplier_max,
                ),
              }))}
            />
            <strong>{Math.round(tuning.global_multiplier * 100)}%</strong>
          </div>
        </label>
        <label>
          <span>PWM 斜率</span>
          <div>
            <input
              type="number"
              min={limits.pwm_slew_rate_min_us_per_s}
              max={limits.pwm_slew_rate_max_us_per_s}
              step={1}
              value={tuning.pwm_slew_rate_us_per_s}
              disabled={!stopped || !tuningReady}
              onChange={event => setTuning(current => ({
                ...current,
                pwm_slew_rate_us_per_s: Math.round(clamp(
                  Number(event.target.value),
                  limits.pwm_slew_rate_min_us_per_s,
                  limits.pwm_slew_rate_max_us_per_s,
                )),
              }))}
            />
            <span>us/s</span>
          </div>
        </label>
        <label>
          <span>命令超时</span>
          <div>
            <input
              type="number"
              min={limits.command_timeout_min_ms}
              max={limits.command_timeout_max_ms}
              step={1}
              value={tuning.command_timeout_ms}
              disabled={!stopped || !tuningReady}
              onChange={event => setTuning(current => ({
                ...current,
                command_timeout_ms: Math.round(clamp(
                  Number(event.target.value),
                  limits.command_timeout_min_ms,
                  limits.command_timeout_max_ms,
                )),
              }))}
            />
            <span>ms</span>
          </div>
        </label>
      </div>

      <div className="axis-control-head">
        <div>
          <h3>六轴点动</h3>
          <p>按住持续发送，松开即发送零指令。按钮输出先经过本页增益、限幅和全局倍率。</p>
        </div>
        <div className="axis-control-state">
          <StatusBadge label={modeActive ? '六轴模式已开启' : '六轴模式未开启'} tone={modeActive ? 'ok' : 'muted'} />
          <StatusBadge
            label={status?.horizontal_saturated ? '水平组已限幅' : '水平组正常'}
            tone={status?.horizontal_saturated ? 'warning' : 'muted'}
          />
          <StatusBadge
            label={status?.vertical_saturated ? '垂直组已限幅' : '垂直组正常'}
            tone={status?.vertical_saturated ? 'warning' : 'muted'}
          />
        </div>
      </div>

      <div className="axis-control-grid">
        {MOTION_AXES.map(axis => (
          <div className="axis-control" key={axis.key}>
            <div className="axis-control__name">
              <strong>{axis.name}</strong>
              <span>{axis.key}</span>
            </div>
            <button
              className={`axis-button axis-button--negative ${heldButton === `${axis.key}:-1` ? 'is-held' : ''}`}
              disabled={!holdAllowed}
              onPointerDown={event => startHold(event, axis.key, -1)}
              onPointerUp={() => releaseHold(true)}
              onPointerCancel={() => releaseHold(false)}
            >
              {axis.negative}
              <small>-100%</small>
            </button>
            <button
              className={`axis-button axis-button--positive ${heldButton === `${axis.key}:1` ? 'is-held' : ''}`}
              disabled={!holdAllowed}
              onPointerDown={event => startHold(event, axis.key, 1)}
              onPointerUp={() => releaseHold(true)}
              onPointerCancel={() => releaseHold(false)}
            >
              {axis.positive}
              <small>+100%</small>
            </button>
          </div>
        ))}
      </div>
    </Panel>
  );
}

function GamepadPanel({
  status,
  connected,
  capabilities,
  capabilitiesReady,
  safetyLocked,
  onError,
  onEvent,
}: {
  status: RobotStatus | null;
  connected: boolean;
  capabilities: Capabilities;
  capabilitiesReady: boolean;
  safetyLocked: boolean;
  onError: (msg: string) => void;
  onEvent: (msg: string) => void;
}) {
  const [busy, setBusy] = useState(false);
  const gamepad = status?.gamepad;
  const active = status?.control_mode === 'GAMEPAD';
  const neutralInput = Boolean(gamepad && Object.values(
    gamepad.mapped_command,
  ).every(value => Math.abs(value) < 1e-7));
  const neutralPwm = Boolean(
    status
    && status.confirmed_pwm.length === capabilities.channel_count
    && status.confirmed_pwm.every(
      value => value === capabilities.pwm.neutral_us),
  );
  const freshInput = Boolean(
    gamepad
    && gamepad.command_age_ms !== null
    && gamepad.command_age_ms < gamepad.zero_timeout_ms,
  );
  const canEnter = Boolean(
    connected
    && capabilitiesReady
    && capabilities.features.gamepad_control
    && status
    && status.stm32_online
    && !status.status_stale
    && status.safety_state === 1
    && status.control_mode === 'IDLE'
    && !safetyLocked
    && gamepad?.client_connected
    && gamepad.lease_active
    && gamepad.gamepad_connected
    && gamepad.control_enabled
    && freshInput
    && neutralInput
    && neutralPwm
    && !busy,
  );

  const readiness = (() => {
    if (!connected || !status) return '等待网页遥测连接';
    if (!capabilitiesReady || !capabilities.features.gamepad_control) {
      return '手柄能力配置不可用';
    }
    if (active) return 'GAMEPAD 正在控制六轴运动';
    if (status.control_mode !== 'IDLE') {
      return `${status.control_mode} 正在占用运动控制`;
    }
    if (!gamepad?.client_connected) return '等待电脑转发程序连接';
    if (!gamepad.gamepad_connected) return '等待 USB 手柄';
    if (!gamepad.control_enabled) return '电脑端“控制开启”尚未打开';
    if (status.safety_state !== 1) return '请先在网页顶部解锁';
    if (!neutralPwm) return '8 路 PWM 尚未确认回中';
    if (!neutralInput) return '请将摇杆回中并松开 A/Y';
    if (!freshInput) return '手柄数据已超时';
    return '可进入 GAMEPAD 模式';
  })();

  const runMode = async (path: string, message: string) => {
    if (busy) return;
    setBusy(true);
    try {
      await apiPost(path);
      onEvent(message);
    } catch (error) {
      onError(
        `${message}失败：${
          error instanceof Error ? error.message : '未知错误'
        }`,
      );
    } finally {
      setBusy(false);
    }
  };

  const axisLabels = [
    'axis 0 左杆左右',
    'axis 1 左杆上下',
    'axis 2 左扳机',
    'axis 3 右杆上下（未使用）',
    'axis 4 右杆左右',
    'axis 5 右扳机',
  ];
  const buttonLabels = ['A', 'B（保留）', 'X（保留）', 'Y'];
  const commandAxes: Array<[MotionAxis, string]> = [
    ['surge', '纵向 surge'],
    ['sway', '横向 sway'],
    ['heave', '垂向 heave'],
    ['roll', '横滚 roll'],
    ['pitch', '俯仰 pitch'],
    ['yaw', '偏航 yaw'],
  ];
  const ageTone = gamepad?.command_age_ms === null
    || gamepad?.command_age_ms === undefined
    ? 'muted'
    : gamepad.command_age_ms >= gamepad.disconnect_timeout_ms
      ? 'danger'
      : gamepad.command_age_ms >= gamepad.zero_timeout_ms
        ? 'warn'
        : 'good';

  return (
    <Panel title="手柄控制" eyebrow="GAMEPAD" className="panel--wide">
      <div className="motion-toolbar">
        <div>
          <div className="control-summary__label">控制状态</div>
          <div className={`control-summary__value ${canEnter || active ? 'is-ready' : ''}`}>
            {readiness}
          </div>
        </div>
        <div className="control-summary__actions">
          {!active && (
            <button
              className="btn btn--primary"
              disabled={!canEnter}
              onClick={() => void runMode(
                '/api/gamepad/enable',
                'GAMEPAD 模式已启用',
              )}
            >
              {busy ? '处理中' : '进入 GAMEPAD'}
            </button>
          )}
          {active && (
            <button
              className="btn btn--warning"
              disabled={busy}
              onClick={() => void runMode(
                '/api/gamepad/disable',
                'GAMEPAD 已退出并上锁',
              )}
            >
              {busy ? '处理中' : '退出并上锁'}
            </button>
          )}
        </div>
      </div>

      {gamepad?.heave_conflict && (
        <div className="inline-warning">
          A 与 Y 同时按下，垂向命令已强制归零。
        </div>
      )}
      {gamepad?.resume_requires_neutral && (
        <div className="inline-warning">
          300ms 输入超时已触发。必须先将全部运动输入回中，才会接受后续非零命令。
        </div>
      )}

      <div className="gamepad-metrics">
        <Metric
          label="电脑转发程序"
          value={gamepad?.client_connected ? '已连接' : '未连接'}
          tone={gamepad?.client_connected ? 'good' : 'danger'}
        />
        <Metric
          label="USB 手柄"
          value={gamepad?.gamepad_connected ? '已连接' : '未连接'}
          tone={gamepad?.gamepad_connected ? 'good' : 'danger'}
        />
        <Metric
          label="电脑端控制"
          value={gamepad?.control_enabled ? '已开启' : '已关闭'}
          tone={gamepad?.control_enabled ? 'good' : 'warn'}
        />
        <Metric
          label="命令年龄"
          value={formatAge(gamepad?.command_age_ms ?? null)}
          hint={`归零 ${gamepad?.zero_timeout_ms ?? 300}ms / 上锁 ${
            gamepad?.disconnect_timeout_ms ?? 1000
          }ms`}
          tone={ageTone}
        />
        <Metric
          label="序号"
          value={gamepad?.last_sequence?.toString() ?? '--'}
          hint={`已转发 ${
            gamepad?.last_forwarded_sequence?.toString() ?? '--'
          }`}
        />
        <Metric
          label="STM32 最近确认"
          value={gamepad?.last_stm32_ack ? '成功' : '无确认'}
          hint={gamepad?.eligibility_reason ?? 'no_gamepad_frame'}
          tone={gamepad?.last_stm32_ack ? 'good' : 'muted'}
        />
      </div>

      <div className="gamepad-data-grid">
        <div className="gamepad-data-block">
          <h3>原始摇杆</h3>
          <div className="gamepad-axis-list">
            {axisLabels.map((label, index) => (
              <div key={label}>
                <span>{label}</span>
                <strong>
                  {formatNumber(gamepad?.axes[index] ?? NaN, 3)}
                </strong>
              </div>
            ))}
          </div>
        </div>
        <div className="gamepad-data-block">
          <h3>原始按钮</h3>
          <div className="gamepad-button-list">
            {buttonLabels.map((label, index) => (
              <span
                key={label}
                className={
                  gamepad?.buttons[index] === 1 ? 'is-pressed' : ''
                }
              >
                button {index} {label}: {gamepad?.buttons[index] ?? 0}
              </span>
            ))}
          </div>
        </div>
      </div>

      <div className="gamepad-command-grid">
        {commandAxes.map(([axis, label]) => (
          <div key={axis}>
            <span>{label}</span>
            <strong>
              {formatNumber(
                gamepad?.mapped_command[axis] ?? 0,
                3,
              )}
            </strong>
          </div>
        ))}
      </div>

      <div className="gamepad-config-strip">
        <span>发送 {capabilities.gamepad.send_hz}Hz</span>
        <span>死区 {capabilities.gamepad.deadzone.toFixed(2)}</span>
        <span>Expo {capabilities.gamepad.expo.toFixed(2)}</span>
        <span>全局倍率 {capabilities.gamepad.global_scale.toFixed(2)}</span>
        <span>
          反向 surge {capabilities.gamepad.surge_invert ? '是' : '否'}
          {' / '}sway {capabilities.gamepad.sway_invert ? '是' : '否'}
          {' / '}yaw {capabilities.gamepad.yaw_invert ? '是' : '否'}
        </span>
      </div>

      {(gamepad?.last_error || gamepad?.last_disconnect_reason) && (
        <div className="inline-error">
          {gamepad.last_error
            ? `最近错误：${gamepad.last_error}`
            : `最近断开：${gamepad.last_disconnect_reason}`}
        </div>
      )}
    </Panel>
  );
}

function PwmOutputGrid({
  status,
  connected,
  capabilities,
}: {
  status: RobotStatus | null;
  connected: boolean;
  capabilities: Capabilities;
}) {
  const channels = Array.from({ length: capabilities.channel_count }, (_, index) => index);
  const outputFresh = Boolean(connected && status?.stm32_online && !status.status_stale);

  return (
    <Panel title="PWM 输出" eyebrow="OUTPUT" className="panel--wide">
      {!outputFresh && (
        <div className="inline-warning">
          PWM 确认状态不可用或已过期；下方只显示最后缓存，不能代表当前实际输出。
        </div>
      )}
      <div className="request-summary">
        <span>请求状态：{status ? requestStateLabel(status.request_state) : '未知'}</span>
        {status?.last_command_error && <span className="request-summary__error">错误：{status.last_command_error}</span>}
        {status?.neutral_reason && <span>最近回中原因：{status.neutral_reason}</span>}
      </div>
      <div className="output-grid">
        {channels.map(index => {
          const confirmed = status?.confirmed_pwm[index];
          const requested = status?.requested_pwm[index];
          const active = Number.isFinite(confirmed)
            && Math.abs((confirmed as number) - capabilities.pwm.neutral_us) >= 1;
          const state = !Number.isFinite(confirmed)
            ? '确认未知'
            : !outputFresh
              ? '过期缓存'
              : active
                ? '确认输出'
                : '确认中位';
          return (
            <div key={index} className={`output-cell ${active ? 'is-active' : ''}`}>
              <div className="output-cell__channel">CH{index}</div>
              <div className="output-cell__value">确认 {formatPwm(confirmed)}</div>
              <div className="output-cell__target">请求 {formatPwm(requested)}</div>
              <div className="output-cell__state">{state}</div>
            </div>
          );
        })}
      </div>
    </Panel>
  );
}

function EventLog({ events }: { events: string[] }) {
  return (
    <Panel title="事件日志" eyebrow="LOG" className="panel--wide">
      <div className="event-log">
        {events.length === 0 ? (
          <div className="empty-state">暂无事件</div>
        ) : (
          events.slice(-60).map((event, index) => <div key={`${event}-${index}`}>{event}</div>)
        )}
      </div>
    </Panel>
  );
}

export default function App() {
  const {
    status,
    connected,
    events,
    addEvent,
    telemetryVersion,
    connectionVersion,
  } = useWebSocket();
  const {
    capabilities,
    ready: capabilitiesReady,
    error: capabilitiesError,
  } = useCapabilities(connectionVersion);
  const [notice, setNotice] = useState<Notice | null>(null);
  const [pendingAction, setPendingAction] = useState<string | null>(null);
  const [estopPending, setEstopPending] = useState(false);
  const [estopGateVersion, setEstopGateVersion] = useState<number | null>(null);
  const [activePage, setActivePage] = useState<
    'dashboard' | 'motion' | 'gamepad'
  >(() => (
    window.location.hash === '#motion'
      ? 'motion'
      : window.location.hash === '#gamepad'
        ? 'gamepad'
        : 'dashboard'
  ));
  const actionInFlight = useRef(false);
  const actionController = useRef<AbortController | null>(null);
  const actionGeneration = useRef(0);
  const estopInFlight = useRef(false);
  const telemetryVersionRef = useRef(telemetryVersion);
  telemetryVersionRef.current = telemetryVersion;
  const linkReady = isLinkReady(connected, status);
  const controlReady = backendReady(connected, status) && capabilitiesReady;

  useEffect(() => {
    const syncPageFromHash = () => {
      setActivePage(
        window.location.hash === '#motion'
          ? 'motion'
          : window.location.hash === '#gamepad'
            ? 'gamepad'
            : 'dashboard',
      );
    };
    window.addEventListener('hashchange', syncPageFromHash);
    return () => window.removeEventListener('hashchange', syncPageFromHash);
  }, []);

  const selectPage = (page: 'dashboard' | 'motion' | 'gamepad') => {
    window.location.hash = page;
    setActivePage(page);
  };

  useEffect(() => {
    if (connected) return;
    actionGeneration.current += 1;
    actionController.current?.abort();
    actionController.current = null;
    actionInFlight.current = false;
    setPendingAction(null);
  }, [connected]);

  useEffect(() => () => {
    actionGeneration.current += 1;
    actionController.current?.abort();
  }, []);

  useEffect(() => {
    if (
      estopGateVersion === null
      || estopPending
      || telemetryVersion <= estopGateVersion
      || !connected
      || !status
      || status.status_stale
    ) return;
    if (status.estop_locked === false && status.safety_state !== 5) {
      setEstopGateVersion(null);
    }
  }, [connected, estopGateVersion, estopPending, status, telemetryVersion]);

  const pwmSafetyLocked = Boolean(
    pendingAction
    || estopPending
    || estopGateVersion !== null
    || status?.estop_locked !== false
    || status?.backend_motion_inhibited !== false
    || status?.safety_state === 5,
  );

  const setError = useCallback(
    (text: string) => setNotice({ kind: 'error', text }),
    [],
  );

  const runAction = async (
    name: string,
    fn: (signal: AbortSignal) => Promise<void>,
  ) => {
    if (actionInFlight.current) return;
    actionInFlight.current = true;
    const controller = new AbortController();
    const generation = ++actionGeneration.current;
    actionController.current = controller;
    setPendingAction(name);
    setNotice(null);
    try {
      await fn(controller.signal);
      if (generation !== actionGeneration.current) return;
      addEvent(`${name} 已发送`);
    } catch (e: unknown) {
      if (controller.signal.aborted) return;
      setError(`${name} 失败：${e instanceof Error ? e.message : '未知错误'}`);
    } finally {
      if (generation === actionGeneration.current) {
        actionController.current = null;
        actionInFlight.current = false;
        setPendingAction(null);
      }
    }
  };

  const doArm = () => runAction('解锁', async signal => { await apiPost('/api/arm', undefined, signal); });
  const doDisarm = () => runAction('上锁', async signal => { await apiPost('/api/disarm', undefined, signal); });
  const doResetEstop = () => runAction('解除急停', async signal => { await apiPost('/api/reset-estop', undefined, signal); });
  const doEstop = () => {
    if (estopInFlight.current) return;
    if (!window.confirm('确认执行急停？\n\n所有 PWM 将回到中位，系统进入急停锁定状态。')) return;
    estopInFlight.current = true;
    setEstopPending(true);
    setEstopGateVersion(telemetryVersion);
    setNotice(null);

    // ESTOP preempts the page's ordinary command request. The backend and
    // STM32 remain authoritative if that earlier request already reached them.
    actionGeneration.current += 1;
    actionController.current?.abort();
    actionController.current = null;
    actionInFlight.current = false;
    setPendingAction(null);

    void apiPost('/api/emergency-stop')
      .then(() => addEvent('急停 已发送'))
      .catch((e: unknown) => {
        setError(`急停失败：${e instanceof Error ? e.message : '未知错误'}`);
      })
      .finally(() => {
        // Require telemetry newer than the completed HTTP request. A status
        // frame received while ESTOP was still in flight cannot unlock motion.
        setEstopGateVersion(telemetryVersionRef.current);
        estopInFlight.current = false;
        setEstopPending(false);
      });
  };

  return (
    <div className="app-shell">
      <Header
        status={status}
        connected={connected}
        backendReady={controlReady}
        linkReady={linkReady}
        motionInhibited={status?.backend_motion_inhibited !== false}
        pendingAction={pendingAction}
        estopPending={estopPending}
        onArm={doArm}
        onDisarm={doDisarm}
        onResetEstop={doResetEstop}
        onEstop={doEstop}
      />

      <main className="workspace">
        <nav className="page-tabs" aria-label="控制台页面">
          <button
            className={activePage === 'dashboard' ? 'is-active' : ''}
            onClick={() => selectPage('dashboard')}
          >
            状态与 PWM
          </button>
          <button
            className={activePage === 'motion' ? 'is-active' : ''}
            onClick={() => selectPage('motion')}
          >
            运动调参
          </button>
          <button
            className={activePage === 'gamepad' ? 'is-active' : ''}
            onClick={() => selectPage('gamepad')}
          >
            手柄控制
          </button>
        </nav>

        <ReadinessStrip
          status={status}
          connected={connected}
          capabilitiesReady={capabilitiesReady}
        />

        {capabilitiesError && (
          <div className="notice notice--error">
            无法加载安全能力配置：{capabilitiesError}。PWM 与模式控制保持锁定。
          </div>
        )}

        {notice && (
          <div className={`notice notice--${notice.kind}`}>
            {notice.text}
          </div>
        )}

        {(status?.safety_state === 5 || estopGateVersion !== null) && (
          <div className="estop-banner">
            {status?.safety_state === 5
              ? '急停已由 STM32 状态确认。所有通道应保持中位，确认安全后才能解除急停。'
              : '急停请求已发起，PWM 控制已在本地立即锁定，等待新的 STM32 状态确认。'}
          </div>
        )}

        {activePage === 'dashboard' ? (
          <div className="dashboard-grid">
            <SystemPanel status={status} connected={connected} />
            <SensorPanel status={status} connected={connected} />
            <PwmPanel
              status={status}
              connected={connected}
              capabilities={capabilities}
              capabilitiesReady={capabilitiesReady}
              safetyLocked={pwmSafetyLocked}
              telemetryVersion={telemetryVersion}
              onError={setError}
              onEvent={addEvent}
            />
            <PwmOutputGrid status={status} connected={connected} capabilities={capabilities} />
            <EventLog events={events} />
          </div>
        ) : activePage === 'motion' ? (
          <div className="dashboard-grid">
            <MotionTuningPanel
              status={status}
              connected={connected}
              capabilities={capabilities}
              capabilitiesReady={capabilitiesReady}
              safetyLocked={pwmSafetyLocked}
              connectionVersion={connectionVersion}
              onError={setError}
              onEvent={addEvent}
            />
            <PwmOutputGrid status={status} connected={connected} capabilities={capabilities} />
            <EventLog events={events} />
          </div>
        ) : (
          <div className="dashboard-grid">
            <GamepadPanel
              status={status}
              connected={connected}
              capabilities={capabilities}
              capabilitiesReady={capabilitiesReady}
              safetyLocked={pwmSafetyLocked}
              onError={setError}
              onEvent={addEvent}
            />
            <PwmOutputGrid
              status={status}
              connected={connected}
              capabilities={capabilities}
            />
            <EventLog events={events} />
          </div>
        )}

        <footer className="footer-strip">
          <span>TX {status?.tx_frames ?? 0}</span>
          <span>RX {status?.rx_frames ?? 0}</span>
          <span>接收错误 {status?.rx_errors ?? 0}</span>
          <span>CRC 错误 {status?.crc_errors ?? 0}</span>
          <span>ACK 超时 {status?.ack_timeouts ?? 0}</span>
          <span>NACK {status?.nack_count ?? 0}</span>
          <span>请求 {status ? requestStateLabel(status.request_state) : '未知'}</span>
          <span>会话 {status?.session_id?.slice(0, 8) ?? '未知'}</span>
          <span>最后更新 {formatTime(status?.last_update ?? 0)}</span>
        </footer>
      </main>
    </div>
  );
}
