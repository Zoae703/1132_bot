export interface PwmCapabilities {
  neutral_us: number;
  min_test_us: number;
  max_test_us: number;
  min_absolute_us: number;
  max_absolute_us: number;
  min_test_duration_ms: number;
  max_test_duration_ms: number;
  default_timeout_ms: number;
}

export interface FeatureCapabilities {
  manual_pwm: boolean;
  motor_mapping: boolean;
  sensor_stream: boolean;
  emergency_stop: boolean;
}

export interface TelemetryCapabilities {
  status_hz: number | null;
  sensors_hz: number | null;
  status_stale_timeout_s: number | null;
  sensors_stale_timeout_s: number | null;
}

export interface Capabilities {
  protocol_version: number;
  channel_count: number;
  pwm: PwmCapabilities;
  features: FeatureCapabilities;
  telemetry: TelemetryCapabilities;
  sensor_poll_hz: number;
}

/**
 * Used only while capabilities are unavailable. The range is deliberately
 * neutral-only, and the UI additionally keeps all motion controls disabled.
 */
export const LOCKED_CAPABILITIES: Capabilities = {
  protocol_version: 2,
  channel_count: 8,
  pwm: {
    neutral_us: 1500,
    min_test_us: 1500,
    max_test_us: 1500,
    min_absolute_us: 1500,
    max_absolute_us: 1500,
    min_test_duration_ms: 200,
    max_test_duration_ms: 200,
    default_timeout_ms: 200,
  },
  features: {
    manual_pwm: false,
    motor_mapping: false,
    sensor_stream: false,
    emergency_stop: true,
  },
  telemetry: {
    status_hz: null,
    sensors_hz: null,
    status_stale_timeout_s: null,
    sensors_stale_timeout_s: null,
  },
  sensor_poll_hz: 0,
};

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function finiteNumber(value: unknown, name: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new Error(`${name} 必须是有限数字`);
  }
  return value;
}

function integer(value: unknown, name: string): number {
  const parsed = finiteNumber(value, name);
  if (!Number.isSafeInteger(parsed)) throw new Error(`${name} 必须是整数`);
  return parsed;
}

function optionalPositive(value: unknown, name: string): number | null {
  if (value === undefined || value === null) return null;
  const parsed = finiteNumber(value, name);
  if (parsed <= 0) throw new Error(`${name} 必须大于 0`);
  return parsed;
}

export function normalizeCapabilities(raw: unknown): Capabilities {
  if (!isRecord(raw) || !isRecord(raw.pwm)) {
    throw new Error('能力配置缺少 pwm 对象');
  }
  const pwm = raw.pwm;
  const protocolVersion = integer(raw.protocol_version, 'protocol_version');
  const channelCount = integer(raw.channel_count, 'channel_count');
  const neutral = integer(pwm.neutral_us, 'pwm.neutral_us');
  const minTest = integer(pwm.min_test_us, 'pwm.min_test_us');
  const maxTest = integer(pwm.max_test_us, 'pwm.max_test_us');
  const minAbsolute = integer(pwm.min_absolute_us, 'pwm.min_absolute_us');
  const maxAbsolute = integer(pwm.max_absolute_us, 'pwm.max_absolute_us');
  const minDuration = integer(pwm.min_test_duration_ms, 'pwm.min_test_duration_ms');
  const maxDuration = integer(pwm.max_test_duration_ms, 'pwm.max_test_duration_ms');
  const defaultDuration = integer(pwm.default_timeout_ms, 'pwm.default_timeout_ms');

  if (protocolVersion !== 2) {
    throw new Error('protocol_version 必须为 2');
  }
  if (channelCount !== 8) {
    throw new Error('channel_count 必须为 8');
  }
  if (neutral !== 1500) {
    throw new Error('PWM 中位必须为 1500us');
  }
  if (!(minAbsolute <= minTest && minTest <= neutral && neutral <= maxTest && maxTest <= maxAbsolute)) {
    throw new Error('PWM 范围顺序无效');
  }
  if (minDuration <= 0 || maxDuration < minDuration || maxDuration > 2000) {
    throw new Error('测试时长范围无效');
  }
  if (defaultDuration < minDuration || defaultDuration > maxDuration) {
    throw new Error('默认测试时长必须在最小与最大测试时长之间');
  }

  const telemetry = isRecord(raw.telemetry) ? raw.telemetry : {};
  if (!isRecord(raw.features)) throw new Error('能力配置缺少 features 对象');
  const features = raw.features;
  const sensorPollHz = finiteNumber(raw.sensor_poll_hz, 'sensor_poll_hz');
  if (sensorPollHz < 0.5 || sensorPollHz > 20) {
    throw new Error('sensor_poll_hz 必须在 0.5..20Hz');
  }
  return {
    protocol_version: protocolVersion,
    channel_count: channelCount,
    pwm: {
      neutral_us: neutral,
      min_test_us: minTest,
      max_test_us: maxTest,
      min_absolute_us: minAbsolute,
      max_absolute_us: maxAbsolute,
      min_test_duration_ms: minDuration,
      max_test_duration_ms: maxDuration,
      default_timeout_ms: defaultDuration,
    },
    features: {
      manual_pwm: features.manual_pwm === true,
      motor_mapping: features.motor_mapping === true,
      sensor_stream: features.sensor_stream === true,
      emergency_stop: features.emergency_stop === true,
    },
    telemetry: {
      status_hz: optionalPositive(telemetry.status_hz, 'telemetry.status_hz'),
      sensors_hz: optionalPositive(telemetry.sensors_hz, 'telemetry.sensors_hz'),
      status_stale_timeout_s: optionalPositive(
        telemetry.status_stale_timeout_s,
        'telemetry.status_stale_timeout_s',
      ),
      sensors_stale_timeout_s: optionalPositive(
        telemetry.sensors_stale_timeout_s,
        'telemetry.sensors_stale_timeout_s',
      ),
    },
    sensor_poll_hz: sensorPollHz,
  };
}
