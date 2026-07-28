import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import type { Capabilities } from './capabilities';
import {
  appendDepthHistory,
  chartPath,
  depthHistoryPoint,
  finiteChartRange,
  normalizeDepthTuningSnapshot,
  type DepthHistoryPoint,
  type DepthPidTuning,
  type DepthTuningSnapshot,
} from './depthControl';
import type { RobotStatus } from './status';

export const KEEPALIVE_INTERVAL_MS = 200;
const HISTORY_WINDOW_SECONDS = 120;

type ApiGet = (path: string, signal?: AbortSignal) => Promise<unknown>;
type ApiPost = (
  path: string,
  body?: unknown,
  signal?: AbortSignal,
) => Promise<unknown>;

interface DepthControlPanelProps {
  status: RobotStatus | null;
  connected: boolean;
  capabilities: Capabilities;
  capabilitiesReady: boolean;
  safetyLocked: boolean;
  telemetryVersion: number;
  connectionVersion: number;
  apiGet: ApiGet;
  apiPost: ApiPost;
  onError: (message: string) => void;
  onEvent: (message: string) => void;
}

const DEFAULT_TUNING: DepthPidTuning = {
  kp: 0,
  ki: 0,
  kd: 0,
  p_limit_us: 0,
  i_limit_us: 0,
  d_limit_us: 0,
  output_limit_us: 0,
};

const TUNING_FIELDS: Array<{
  key: keyof DepthPidTuning;
  label: string;
  unit: string;
  step: number;
  range: (
    capabilities: Capabilities['depth_pid'],
  ) => [number, number];
}> = [
  {
    key: 'kp',
    label: 'Kp',
    unit: 'us/cm',
    step: 0.01,
    range: limits => [limits.kp_min, limits.kp_max],
  },
  {
    key: 'ki',
    label: 'Ki',
    unit: 'us/(cm·采样)',
    step: 0.01,
    range: limits => [limits.ki_min, limits.ki_max],
  },
  {
    key: 'kd',
    label: 'Kd',
    unit: 'us·采样/cm',
    step: 0.01,
    range: limits => [limits.kd_min, limits.kd_max],
  },
  {
    key: 'p_limit_us',
    label: 'P 项限幅',
    unit: 'us',
    step: 1,
    range: limits => [
      limits.term_limit_min_us,
      limits.term_limit_max_us,
    ],
  },
  {
    key: 'i_limit_us',
    label: 'I 项限幅',
    unit: 'us',
    step: 1,
    range: limits => [
      limits.term_limit_min_us,
      limits.term_limit_max_us,
    ],
  },
  {
    key: 'd_limit_us',
    label: 'D 项限幅',
    unit: 'us',
    step: 1,
    range: limits => [
      limits.term_limit_min_us,
      limits.term_limit_max_us,
    ],
  },
  {
    key: 'output_limit_us',
    label: '总输出限幅',
    unit: 'us',
    step: 1,
    range: limits => [
      limits.output_limit_min_us,
      limits.output_limit_max_us,
    ],
  },
];

function finite(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

function record(value: unknown): Record<string, unknown> | null {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
    ? value as Record<string, unknown>
    : null;
}

function controlPayload(value: unknown): Record<string, unknown> | null {
  const response = record(value);
  return record(response?.control) ?? response;
}

function format(value: number, precision = 2, unit = '') {
  return Number.isFinite(value) ? `${value.toFixed(precision)}${unit}` : '--';
}

function formatAge(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '未知';
  return value < 1000 ? `${Math.round(value)}ms` : `${(value / 1000).toFixed(1)}s`;
}

function formatReportTime(value: number) {
  if (!Number.isFinite(value) || value <= 0) return '从未';
  const milliseconds = value > 100_000_000_000 ? value : value * 1000;
  if (milliseconds < 1_000_000_000_000) return `${value.toFixed(3)}s`;
  return new Date(milliseconds).toLocaleTimeString('zh-CN', { hour12: false });
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

function ChartGrid() {
  return (
    <g className="depth-chart__grid">
      {[0, 1, 2, 3, 4].map(index => (
        <line key={`h-${index}`} x1="0" x2="800" y1={index * 55} y2={index * 55} />
      ))}
      {[0, 1, 2, 3, 4].map(index => (
        <line key={`v-${index}`} y1="0" y2="220" x1={index * 200} x2={index * 200} />
      ))}
    </g>
  );
}

function DepthCharts({ history }: { history: DepthHistoryPoint[] }) {
  const paths = useMemo(() => {
    if (history.length === 0) return null;
    const latest = history[history.length - 1]?.timestamp_s ?? 0;
    const xMin = latest - HISTORY_WINDOW_SECONDS;
    const [depthMin, depthMax] = finiteChartRange(
      history.flatMap(point => [point.actual_m, point.target_m]),
      0.2,
    );
    const [errorMin, errorMax] = finiteChartRange(
      history.map(point => point.error_m),
      0.1,
    );
    const [outputMin, outputMax] = finiteChartRange(
      history.flatMap(point => [
        point.output_us,
        point.p_us,
        point.i_us,
        point.d_us,
      ]),
      20,
    );
    return {
      actual: chartPath(
        history, 'actual_m', xMin, latest, depthMin, depthMax,
      ),
      target: chartPath(
        history, 'target_m', xMin, latest, depthMin, depthMax,
      ),
      error: chartPath(
        history, 'error_m', xMin, latest, errorMin, errorMax,
      ),
      output: chartPath(
        history, 'output_us', xMin, latest, outputMin, outputMax,
      ),
      p: chartPath(history, 'p_us', xMin, latest, outputMin, outputMax),
      i: chartPath(history, 'i_us', xMin, latest, outputMin, outputMax),
      d: chartPath(history, 'd_us', xMin, latest, outputMin, outputMax),
      depthRange: `${depthMin.toFixed(2)}..${depthMax.toFixed(2)}m`,
      errorRange: `${errorMin.toFixed(2)}..${errorMax.toFixed(2)}m`,
      outputRange: `${outputMin.toFixed(0)}..${outputMax.toFixed(0)}us`,
    };
  }, [history]);

  return (
    <div className="depth-charts">
      <div className="depth-chart">
        <div className="depth-chart__head">
          <strong>实际深度 / 目标深度</strong>
          <span>{paths?.depthRange ?? '--'} · 最近 120 秒</span>
        </div>
        {paths ? (
          <svg
            viewBox="0 0 800 220"
            preserveAspectRatio="none"
            role="img"
            aria-label="最近120秒实际深度和目标深度曲线"
          >
            <ChartGrid />
            <path className="depth-chart__line depth-chart__line--actual" d={paths.actual} />
            <path className="depth-chart__line depth-chart__line--target" d={paths.target} />
          </svg>
        ) : (
          <div className="empty-state">等待有效且新鲜的定深遥测</div>
        )}
        <div className="depth-chart__legend">
          <span className="is-actual">实际深度</span>
          <span className="is-target">目标深度</span>
        </div>
      </div>

      <div className="depth-chart">
        <div className="depth-chart__head">
          <strong>误差 / PID 输出</strong>
          <span>
            误差 {paths?.errorRange ?? '--'} · 输出 {paths?.outputRange ?? '--'}
          </span>
        </div>
        {paths ? (
          <svg
            viewBox="0 0 800 220"
            preserveAspectRatio="none"
            role="img"
            aria-label="最近120秒深度误差和PID输出曲线"
          >
            <ChartGrid />
            <path className="depth-chart__line depth-chart__line--error" d={paths.error} />
            <path className="depth-chart__line depth-chart__line--output" d={paths.output} />
            <path className="depth-chart__line depth-chart__line--p" d={paths.p} />
            <path className="depth-chart__line depth-chart__line--i" d={paths.i} />
            <path className="depth-chart__line depth-chart__line--d" d={paths.d} />
          </svg>
        ) : (
          <div className="empty-state">等待有效且新鲜的 PID 遥测</div>
        )}
        <div className="depth-chart__legend">
          <span className="is-error">误差</span>
          <span className="is-output">总输出</span>
          <span className="is-p">P</span>
          <span className="is-i">I</span>
          <span className="is-d">D</span>
        </div>
      </div>
    </div>
  );
}

export default function DepthControlPanel({
  status,
  connected,
  capabilities,
  capabilitiesReady,
  safetyLocked,
  telemetryVersion,
  connectionVersion,
  apiGet,
  apiPost,
  onError,
  onEvent,
}: DepthControlPanelProps) {
  const [tuning, setTuning] = useState<DepthPidTuning>(DEFAULT_TUNING);
  const [snapshot, setSnapshot] = useState<DepthTuningSnapshot | null>(null);
  const [targetDepth, setTargetDepth] = useState(0);
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [controlBusy, setControlBusy] = useState(false);
  const [leaseOwned, setLeaseOwned] = useState(false);
  const [leaseWarning, setLeaseWarning] = useState<string | null>(null);
  const [history, setHistory] = useState<DepthHistoryPoint[]>([]);
  const leaseOwnedRef = useRef(false);
  const leaseTargetRef = useRef(0);
  const keepaliveTimerRef = useRef<number | null>(null);
  const keepaliveAbortRef = useRef<AbortController | null>(null);
  const keepaliveInFlightRef = useRef(false);
  const leaseEpochRef = useRef(0);

  const setOwned = useCallback((owned: boolean) => {
    leaseOwnedRef.current = owned;
    setLeaseOwned(owned);
  }, []);

  const haltKeepalive = useCallback((warning?: string) => {
    leaseEpochRef.current += 1;
    if (keepaliveTimerRef.current !== null) {
      window.clearInterval(keepaliveTimerRef.current);
      keepaliveTimerRef.current = null;
    }
    keepaliveAbortRef.current?.abort();
    keepaliveAbortRef.current = null;
    keepaliveInFlightRef.current = false;
    setOwned(false);
    if (warning) setLeaseWarning(warning);
  }, [setOwned]);

  const keepaliveTimeoutMs = Math.max(
    100,
    Math.min(500, capabilities.depth_pid.lease_timeout_ms - 50),
  );

  const sendKeepalive = useCallback(async (epoch: number) => {
    if (
      epoch !== leaseEpochRef.current
      || !leaseOwnedRef.current
      || keepaliveInFlightRef.current
    ) return;
    keepaliveInFlightRef.current = true;
    const controller = new AbortController();
    keepaliveAbortRef.current = controller;
    const timeout = window.setTimeout(
      () => controller.abort(),
      keepaliveTimeoutMs,
    );
    try {
      await apiPost(
        '/api/depth/keepalive',
        { target_depth_m: leaseTargetRef.current },
        controller.signal,
      );
    } catch (error) {
      if (epoch !== leaseEpochRef.current) return;
      const reason = error instanceof Error ? error.message : '未知错误';
      const warning = `定深保活失败：${reason}。已停止继续发送，后端应在 ${
        capabilities.depth_pid.lease_timeout_ms
      }ms 租约到期后安全退出定深。`;
      haltKeepalive(warning);
      onError(warning);
    } finally {
      window.clearTimeout(timeout);
      if (keepaliveAbortRef.current === controller) {
        keepaliveAbortRef.current = null;
      }
      keepaliveInFlightRef.current = false;
    }
  }, [
    apiPost,
    capabilities.depth_pid.lease_timeout_ms,
    haltKeepalive,
    keepaliveTimeoutMs,
    onError,
  ]);

  const beginKeepalive = useCallback((target: number) => {
    haltKeepalive();
    setLeaseWarning(null);
    leaseTargetRef.current = target;
    setOwned(true);
    const epoch = leaseEpochRef.current;
    keepaliveTimerRef.current = window.setInterval(
      () => void sendKeepalive(epoch),
      KEEPALIVE_INTERVAL_MS,
    );
  }, [haltKeepalive, sendKeepalive, setOwned]);

  const applySnapshot = useCallback((value: DepthTuningSnapshot) => {
    setSnapshot(value);
    setTuning({ ...value.desired });
  }, []);

  const reload = useCallback(async (announce = true) => {
    if (
      !connected
      || !capabilitiesReady
      || !capabilities.features.depth_hold
    ) return;
    setLoading(true);
    try {
      const [tuningValue, controlValue] = await Promise.all([
        apiGet('/api/depth/tuning'),
        apiGet('/api/depth/control'),
      ]);
      applySnapshot(normalizeDepthTuningSnapshot(tuningValue));
      const control = controlPayload(controlValue);
      const requested = finite(control?.depth_requested_target_m);
      const active = finite(control?.depth_active_setpoint_m);
      if (requested !== null) setTargetDepth(requested);
      else if (active !== null) setTargetDepth(active);
      if (announce) onEvent('定深参数与控制状态已重新加载');
    } catch (error) {
      onError(`加载定深配置失败：${
        error instanceof Error ? error.message : '未知错误'
      }`);
    } finally {
      setLoading(false);
    }
  }, [
    apiGet,
    applySnapshot,
    capabilities.features.depth_hold,
    capabilitiesReady,
    connected,
    onError,
    onEvent,
  ]);

  useEffect(() => {
    if (
      !connected
      || !capabilitiesReady
      || !capabilities.features.depth_hold
    ) return;
    const controller = new AbortController();
    setLoading(true);
    void Promise.all([
      apiGet('/api/depth/tuning', controller.signal),
      apiGet('/api/depth/control', controller.signal),
    ])
      .then(([tuningValue, controlValue]) => {
        applySnapshot(normalizeDepthTuningSnapshot(tuningValue));
        const control = controlPayload(controlValue);
        const requested = finite(control?.depth_requested_target_m);
        const active = finite(control?.depth_active_setpoint_m);
        if (requested !== null) setTargetDepth(requested);
        else if (active !== null) setTargetDepth(active);
      })
      .catch(error => {
        if (!controller.signal.aborted) {
          onError(`加载定深配置失败：${
            error instanceof Error ? error.message : '未知错误'
          }`);
        }
      })
      .finally(() => {
        if (!controller.signal.aborted) setLoading(false);
      });
    return () => controller.abort();
  }, [
    apiGet,
    applySnapshot,
    capabilities.features.depth_hold,
    capabilitiesReady,
    connected,
    connectionVersion,
    onError,
  ]);

  useEffect(() => {
    setHistory([]);
  }, [connectionVersion, status?.session_id]);

  useEffect(() => {
    if (!status) return;
    const point = depthHistoryPoint(status);
    if (!point) return;
    setHistory(current => appendDepthHistory(current, point));
  }, [status, telemetryVersion]);

  useEffect(() => {
    if (!leaseOwnedRef.current) return;
    if (
      !connected
      || !status
      || status.status_stale
      || status.estop_locked
      || status.backend_motion_inhibited
      || status.safety_state === 5
      || status.safety_state === 6
    ) {
      haltKeepalive(
        '定深保活因连接或安全状态变化已停止；等待后端租约到期并确认退出。',
      );
    }
  }, [
    connected,
    haltKeepalive,
    status,
    telemetryVersion,
  ]);

  useEffect(() => {
    haltKeepalive();
    setLeaseWarning(null);
  }, [connectionVersion, haltKeepalive, status?.session_id]);

  useEffect(() => () => {
    const shouldDisable = leaseOwnedRef.current;
    haltKeepalive();
    if (shouldDisable) {
      void apiPost('/api/depth/disable').catch(() => undefined);
    }
  }, [apiPost, haltKeepalive]);

  const limits = capabilities.depth_pid;
  const targetValid = Number.isFinite(targetDepth)
    && targetDepth >= limits.target_depth_min_m
    && targetDepth <= limits.target_depth_max_m;
  const linkAndFeatureReady = Boolean(
    connected
    && capabilitiesReady
    && capabilities.features.depth_hold
    && status
    && status.stm32_online
    && !status.status_stale,
  );
  const stoppedForTuning = Boolean(
    status
    && (status.safety_state === 0 || status.safety_state === 1)
    && !status.float_enabled,
  );
  const canSave = Boolean(
    linkAndFeatureReady
    && stoppedForTuning
    && snapshot !== null
    && !safetyLocked
    && !loading
    && !saving
    && !controlBusy,
  );
  const canEnable = Boolean(
    linkAndFeatureReady
    && status?.safety_state === 1
    && status.control_mode === 'IDLE'
    && !status.sensors_stale
    && status.depth_sensor_ready
    && status.depth_sample_valid
    && status.depth_actuator_ready
    && status.depth_pid_tuning_synced
    && Number.isFinite(status.depth_m)
    && status.depth_m >= limits.target_depth_min_m
    && status.depth_m <= limits.target_depth_max_m
    && snapshot !== null
    && !safetyLocked
    && !controlBusy
    && targetValid
  );
  const canSetTarget = Boolean(
    linkAndFeatureReady
    && leaseOwned
    && status?.float_enabled
    && !safetyLocked
    && !controlBusy
    && targetValid
  );
  const canDisable = Boolean(
    linkAndFeatureReady
    && (leaseOwned || status?.float_enabled)
    && !controlBusy
  );
  const tuningSynced = status?.depth_pid_tuning_synced
    ?? snapshot?.synced
    ?? false;
  const syncState = status?.depth_pid_tuning_sync_state
    ?? snapshot?.sync_state
    ?? 'pending';
  const syncError = status?.depth_pid_tuning_sync_error
    ?? snapshot?.sync_error;
  const unmanagedActive = Boolean(status?.float_enabled && !leaseOwned);

  const updateTuning = (
    key: keyof DepthPidTuning,
    value: number,
    min: number,
    max: number,
  ) => {
    if (!Number.isFinite(value)) return;
    setTuning(current => ({
      ...current,
      [key]: Math.min(max, Math.max(min, value)),
    }));
  };

  const saveTuning = async () => {
    if (!canSave) return;
    setSaving(true);
    try {
      const value = normalizeDepthTuningSnapshot(
        await apiPost('/api/depth/tuning', tuning),
      );
      applySnapshot(value);
      onEvent('定深 PID 参数已保存并读回');
    } catch (error) {
      onError(`保存定深 PID 参数失败：${
        error instanceof Error ? error.message : '未知错误'
      }`);
    } finally {
      setSaving(false);
    }
  };

  const enableDepthHold = async () => {
    if (!canEnable) return;
    if (!window.confirm(
      `确认启用定深闭环？\n\n目标深度 ${targetDepth.toFixed(2)}m。`
      + ' 页面将每 200ms 发送保活；关闭或断开后保活停止。',
    )) return;
    setControlBusy(true);
    setLeaseWarning(null);
    try {
      await apiPost('/api/depth/enable', { target_depth_m: targetDepth });
      beginKeepalive(targetDepth);
      onEvent(`定深已启用，目标 ${targetDepth.toFixed(2)}m`);
    } catch (error) {
      haltKeepalive();
      onError(`启用定深失败：${
        error instanceof Error ? error.message : '未知错误'
      }`);
    } finally {
      setControlBusy(false);
    }
  };

  const setActiveTarget = async () => {
    if (!canSetTarget) return;
    setControlBusy(true);
    // Stop and abort the interval before changing the target so an older
    // keepalive request cannot restore the previous setpoint afterward.
    haltKeepalive();
    leaseTargetRef.current = targetDepth;
    try {
      await apiPost('/api/depth/target', { target_depth_m: targetDepth });
      beginKeepalive(targetDepth);
      onEvent(`定深目标已更新为 ${targetDepth.toFixed(2)}m`);
    } catch (error) {
      const warning = `更新定深目标失败：${
        error instanceof Error ? error.message : '未知错误'
      }。本地保活已停止，等待后端租约超时并确认状态。`;
      haltKeepalive(warning);
      onError(warning);
    } finally {
      setControlBusy(false);
    }
  };

  const disableDepthHold = async () => {
    if (!canDisable) return;
    haltKeepalive();
    setControlBusy(true);
    try {
      await apiPost('/api/depth/disable');
      setLeaseWarning(null);
      onEvent('定深已停止');
    } catch (error) {
      const warning = `停止定深请求失败：${
        error instanceof Error ? error.message : '未知错误'
      }。本地保活已停止，等待租约超时。`;
      setLeaseWarning(warning);
      onError(warning);
    } finally {
      setControlBusy(false);
    }
  };

  return (
    <section className="panel panel--wide depth-panel">
      <div className="panel__head">
        <div>
          <div className="panel__eyebrow">DEPTH HOLD</div>
          <h2>水下定深闭环调试</h2>
        </div>
        <div className="depth-panel__actions">
          <button
            className="btn btn--secondary"
            disabled={loading || !linkAndFeatureReady}
            onClick={() => void reload()}
          >
            {loading ? '读取中' : '重新读取'}
          </button>
          <button
            className="btn btn--primary"
            disabled={!canEnable}
            onClick={() => void enableDepthHold()}
          >
            {controlBusy ? '处理中' : '显式启用定深'}
          </button>
          <button
            className="btn btn--warning"
            disabled={!canDisable}
            onClick={() => void disableDepthHold()}
          >
            停止定深
          </button>
        </div>
      </div>

      {!capabilities.features.depth_hold && (
        <div className="inline-error">
          后端未声明 depth_hold 能力，定深控制保持锁定。
        </div>
      )}
      {leaseWarning && <div className="active-warning">{leaseWarning}</div>}
      {unmanagedActive && (
        <div className="inline-info">
          后端报告定深已启用，但本页面没有本会话租约；不会自动接管或发送保活，
          只能显式停止。
        </div>
      )}

      <div className="depth-metrics">
        <Metric
          label="闭环状态"
          value={status?.float_enabled ? '已启用' : '已停止'}
          hint={leaseOwned ? '本页面每 200ms 保活' : '无本地保活'}
          tone={status?.float_enabled ? leaseOwned ? 'good' : 'warn' : 'muted'}
        />
        <Metric
          label="深度传感器"
          value={status?.depth_sensor_ready ? '就绪' : '未就绪'}
          hint={`样本 ${status?.depth_sample_valid ? '有效' : '无效'} / ${
            formatAge(status?.depth_sample_age_ms ?? null)
          }`}
          tone={
            status?.depth_sensor_ready && status.depth_sample_valid
              ? 'good'
              : 'danger'
          }
        />
        <Metric
          label="垂向执行器"
          value={status?.depth_actuator_ready ? '就绪' : '未就绪'}
          tone={status?.depth_actuator_ready ? 'good' : 'danger'}
        />
        <Metric
          label="PID 参数同步"
          value={tuningSynced ? '已同步' : '未同步'}
          hint={syncError ?? syncState}
          tone={tuningSynced ? 'good' : 'warn'}
        />
        <Metric
          label="实际深度"
          value={format(status?.depth_m ?? NaN, 3, 'm')}
          hint={status?.sensors_stale ? '传感器数据已过期' : 'FRD Z+ 向下'}
          tone={status?.sensors_stale ? 'danger' : 'normal'}
        />
        <Metric
          label="请求 / 生效目标"
          value={`${format(status?.depth_requested_target_m ?? NaN, 2, 'm')} / ${
            format(status?.depth_active_setpoint_m ?? NaN, 2, 'm')
          }`}
        />
        <Metric
          label="深度误差"
          value={format(status?.depth_error_m ?? NaN, 3, 'm')}
        />
        <Metric
          label="PID 总输出"
          value={format(status?.depth_pid_output_us ?? NaN, 1, 'us')}
          hint={`P ${format(status?.depth_pid_p_us ?? NaN, 1)} / I ${
            format(status?.depth_pid_i_us ?? NaN, 1)
          } / D ${format(status?.depth_pid_d_us ?? NaN, 1)}`}
          tone={status?.depth_pid_saturated ? 'warn' : 'normal'}
        />
        <Metric
          label="输出限幅"
          value={status?.depth_pid_saturated ? '已饱和' : '未饱和'}
          tone={status?.depth_pid_saturated ? 'warn' : 'good'}
        />
        <Metric
          label="最后控制报告"
          value={formatReportTime(status?.last_depth_control_report_at ?? 0)}
        />
      </div>

      <div className="depth-target">
        <label>
          <span>目标深度</span>
          <input
            type="number"
            min={limits.target_depth_min_m}
            max={limits.target_depth_max_m}
            step="0.01"
            value={targetDepth}
            disabled={!linkAndFeatureReady || controlBusy}
            onChange={event => setTargetDepth(Number(event.target.value))}
          />
          <span>m</span>
        </label>
        <small>
          范围 {limits.target_depth_min_m.toFixed(2)}..
          {limits.target_depth_max_m.toFixed(2)}m
        </small>
        <button
          className="btn btn--secondary"
          disabled={!canSetTarget}
          onClick={() => void setActiveTarget()}
        >
          更新生效目标
        </button>
      </div>

      <div className="depth-tuning">
        <div className="depth-tuning__head">
          <div>
            <strong>PID 参数</strong>
            <span>仅 DISARMED / ARMED_IDLE 且定深未启用时允许保存</span>
          </div>
          <button
            className="btn btn--secondary"
            disabled={!canSave}
            onClick={() => void saveTuning()}
          >
            {saving ? '保存中' : '保存并读回'}
          </button>
        </div>
        <div className="depth-tuning__grid">
          {TUNING_FIELDS.map(field => {
            const [min, max] = field.range(limits);
            return (
              <label key={field.key}>
                <span>{field.label}</span>
                <input
                  type="number"
                  value={tuning[field.key]}
                  min={min}
                  max={max}
                  step={field.step}
                  disabled={!canSave}
                  onChange={event => updateTuning(
                    field.key,
                    Number(event.target.value),
                    min,
                    max,
                  )}
                />
                <small>
                  {field.unit} · 范围 {min}..{max} · 回读{' '}
                  {snapshot?.confirmed
                    ? snapshot.confirmed[field.key]
                    : '--'}
                </small>
              </label>
            );
          })}
        </div>
      </div>

      <DepthCharts history={history} />
    </section>
  );
}
