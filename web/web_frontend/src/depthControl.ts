export interface DepthPidTuning {
  kp: number;
  ki: number;
  kd: number;
  p_limit_us: number;
  i_limit_us: number;
  d_limit_us: number;
  output_limit_us: number;
}

export interface DepthTuningSnapshot {
  desired: DepthPidTuning;
  confirmed: DepthPidTuning | null;
  synced: boolean;
  sync_state: string;
  sync_error: string | null;
}

export interface DepthHistorySource {
  status_stale: boolean;
  sensors_stale: boolean;
  depth_sensor_ready: boolean;
  depth_sample_valid: boolean;
  depth_sample_age_ms: number | null;
  depth_m: number;
  depth_active_setpoint_m: number;
  depth_error_m: number;
  depth_pid_output_us: number;
  depth_pid_p_us: number;
  depth_pid_i_us: number;
  depth_pid_d_us: number;
  last_depth_control_report_at: number;
}

export interface DepthHistoryPoint {
  timestamp_s: number;
  actual_m: number;
  target_m: number;
  error_m: number;
  output_us: number;
  p_us: number;
  i_us: number;
  d_us: number;
}

type HistoryValueField = Exclude<keyof DepthHistoryPoint, 'timestamp_s'>;

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function tuningValues(raw: unknown): DepthPidTuning | null {
  if (!isRecord(raw)) return null;
  const value = (key: keyof DepthPidTuning) => (
    typeof raw[key] === 'number' && Number.isFinite(raw[key])
      ? raw[key] as number
      : NaN
  );
  const tuning: DepthPidTuning = {
    kp: value('kp'),
    ki: value('ki'),
    kd: value('kd'),
    p_limit_us: value('p_limit_us'),
    i_limit_us: value('i_limit_us'),
    d_limit_us: value('d_limit_us'),
    output_limit_us: value('output_limit_us'),
  };
  return Object.values(tuning).every(Number.isFinite) ? tuning : null;
}

/**
 * Accept the desired/confirmed response used by the backend. A direct tuning
 * object is also accepted for compatibility with a minimal POST response.
 */
export function normalizeDepthTuningSnapshot(raw: unknown): DepthTuningSnapshot {
  if (!isRecord(raw)) throw new Error('定深参数响应格式无效');
  const wrapped = Object.prototype.hasOwnProperty.call(raw, 'desired');
  const desired = tuningValues(wrapped ? raw.desired : raw);
  const confirmed = wrapped
    ? raw.confirmed === null
      ? null
      : tuningValues(raw.confirmed)
    : desired;
  if (!desired || (wrapped && raw.confirmed !== null && !confirmed)) {
    throw new Error('定深参数响应缺少 PID 数据');
  }
  return {
    desired,
    confirmed,
    synced: wrapped ? raw.synced === true : true,
    sync_state: typeof raw.sync_state === 'string'
      ? raw.sync_state
      : wrapped ? 'unknown' : 'synced',
    sync_error: typeof raw.sync_error === 'string' && raw.sync_error.trim()
      ? raw.sync_error
      : null,
  };
}

function secondsTimestamp(value: number): number {
  return value > 100_000_000_000 ? value / 1000 : value;
}

/**
 * Invalid, stale, or not-yet-ready measurements never enter graph history.
 */
export function depthHistoryPoint(
  source: DepthHistorySource,
): DepthHistoryPoint | null {
  if (
    source.status_stale
    || source.sensors_stale
    || !source.depth_sensor_ready
    || !source.depth_sample_valid
    || source.depth_sample_age_ms === null
    || !Number.isFinite(source.depth_sample_age_ms)
    || source.depth_sample_age_ms < 0
  ) {
    return null;
  }
  const point: DepthHistoryPoint = {
    timestamp_s: secondsTimestamp(source.last_depth_control_report_at),
    actual_m: source.depth_m,
    target_m: source.depth_active_setpoint_m,
    error_m: source.depth_error_m,
    output_us: source.depth_pid_output_us,
    p_us: source.depth_pid_p_us,
    i_us: source.depth_pid_i_us,
    d_us: source.depth_pid_d_us,
  };
  return Object.values(point).every(Number.isFinite) && point.timestamp_s > 0
    ? point
    : null;
}

export function appendDepthHistory(
  history: DepthHistoryPoint[],
  point: DepthHistoryPoint,
  windowSeconds = 120,
): DepthHistoryPoint[] {
  if (!Number.isFinite(windowSeconds) || windowSeconds <= 0) return [point];
  const last = history[history.length - 1];
  if (last && point.timestamp_s < last.timestamp_s) return [point];
  const next = last && point.timestamp_s === last.timestamp_s
    ? [...history.slice(0, -1), point]
    : [...history, point];
  const cutoff = point.timestamp_s - windowSeconds;
  return next.filter(item => item.timestamp_s >= cutoff);
}

export function finiteChartRange(
  values: number[],
  minimumSpan: number,
): [number, number] {
  const finite = values.filter(Number.isFinite);
  if (finite.length === 0) return [-minimumSpan / 2, minimumSpan / 2];
  let min = Math.min(...finite);
  let max = Math.max(...finite);
  const span = Math.max(max - min, minimumSpan);
  const padding = span * 0.12;
  const center = (min + max) / 2;
  min = center - span / 2 - padding;
  max = center + span / 2 + padding;
  return [min, max];
}

export function chartPath(
  points: DepthHistoryPoint[],
  field: HistoryValueField,
  xMin: number,
  xMax: number,
  yMin: number,
  yMax: number,
  width = 800,
  height = 220,
): string {
  if (
    points.length === 0
    || ![xMin, xMax, yMin, yMax, width, height].every(Number.isFinite)
    || xMax <= xMin
    || yMax <= yMin
    || width <= 0
    || height <= 0
  ) return '';
  const x = (timestamp: number) => (
    ((timestamp - xMin) / (xMax - xMin)) * width
  );
  const y = (value: number) => (
    height - ((value - yMin) / (yMax - yMin)) * height
  );
  return points
    .filter(point => (
      Number.isFinite(point.timestamp_s)
      && Number.isFinite(point[field])
    ))
    .map((point, index) => (
      `${index === 0 ? 'M' : 'L'}${x(point.timestamp_s).toFixed(2)},${
        y(point[field]).toFixed(2)
      }`
    ))
    .join(' ');
}
