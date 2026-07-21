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
  const canArm = Boolean(status && status.safety_state === 0 && status.estop_locked === false);
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
  const canTest = availability.canTest && capabilities.features.manual_pwm;
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
            <button className="btn btn--primary" disabled={!motionReady || !capabilitiesReady || safetyLocked || testing} onClick={enterManual}>
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
          const active = Number.isFinite(pwm) && Math.abs(pwm - capabilities.pwm.neutral_us) >= 5;
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
          <input
            type="range"
            min={capabilities.pwm.min_test_us}
            max={capabilities.pwm.max_test_us}
            step={5}
            value={pwmValue}
            disabled={!canTest}
            onChange={event => {
              const value = Number(event.target.value);
              if (Number.isFinite(value)) setPwmValue(value);
            }}
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
            && Math.abs((confirmed as number) - capabilities.pwm.neutral_us) >= 5;
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
  const actionInFlight = useRef(false);
  const actionController = useRef<AbortController | null>(null);
  const actionGeneration = useRef(0);
  const estopInFlight = useRef(false);
  const telemetryVersionRef = useRef(telemetryVersion);
  telemetryVersionRef.current = telemetryVersion;
  const linkReady = isLinkReady(connected, status);
  const controlReady = backendReady(connected, status) && capabilitiesReady;

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

  const setError = (text: string) => setNotice({ kind: 'error', text });

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
