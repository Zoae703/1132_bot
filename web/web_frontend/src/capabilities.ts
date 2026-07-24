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
  motion_tuning: boolean;
  sensor_stream: boolean;
  emergency_stop: boolean;
}

export interface MotionTuningCapabilities {
  axis_order: string[];
  gain_min: number;
  gain_max: number;
  axis_max_output_min: number;
  axis_max_output_max: number;
  global_multiplier_min: number;
  global_multiplier_max: number;
  pwm_slew_rate_min_us_per_s: number;
  pwm_slew_rate_max_us_per_s: number;
  command_timeout_min_ms: number;
  command_timeout_max_ms: number;
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
  motion_tuning: MotionTuningCapabilities;
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
    motion_tuning: false,
    sensor_stream: false,
    emergency_stop: true,
  },
  motion_tuning: {
    axis_order: ['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw'],
    gain_min: 0,
    gain_max: 0,
    axis_max_output_min: 0,
    axis_max_output_max: 0,
    global_multiplier_min: 0,
    global_multiplier_max: 0,
    pwm_slew_rate_min_us_per_s: 100,
    pwm_slew_rate_max_us_per_s: 100,
    command_timeout_min_ms: 200,
    command_timeout_max_ms: 200,
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
  if (!isRecord(raw.motion_tuning)) {
    throw new Error('能力配置缺少 motion_tuning 对象');
  }
  const motionTuning = raw.motion_tuning;
  if (
    !Array.isArray(motionTuning.axis_order)
    || motionTuning.axis_order.length !== 6
    || motionTuning.axis_order.some(value => typeof value !== 'string')
  ) {
    throw new Error('motion_tuning.axis_order 必须包含六轴');
  }
  const normalizedMotionTuning: MotionTuningCapabilities = {
    axis_order: [...motionTuning.axis_order] as string[],
    gain_min: finiteNumber(motionTuning.gain_min, 'motion_tuning.gain_min'),
    gain_max: finiteNumber(motionTuning.gain_max, 'motion_tuning.gain_max'),
    axis_max_output_min: finiteNumber(
      motionTuning.axis_max_output_min,
      'motion_tuning.axis_max_output_min',
    ),
    axis_max_output_max: finiteNumber(
      motionTuning.axis_max_output_max,
      'motion_tuning.axis_max_output_max',
    ),
    global_multiplier_min: finiteNumber(
      motionTuning.global_multiplier_min,
      'motion_tuning.global_multiplier_min',
    ),
    global_multiplier_max: finiteNumber(
      motionTuning.global_multiplier_max,
      'motion_tuning.global_multiplier_max',
    ),
    pwm_slew_rate_min_us_per_s: integer(
      motionTuning.pwm_slew_rate_min_us_per_s,
      'motion_tuning.pwm_slew_rate_min_us_per_s',
    ),
    pwm_slew_rate_max_us_per_s: integer(
      motionTuning.pwm_slew_rate_max_us_per_s,
      'motion_tuning.pwm_slew_rate_max_us_per_s',
    ),
    command_timeout_min_ms: integer(
      motionTuning.command_timeout_min_ms,
      'motion_tuning.command_timeout_min_ms',
    ),
    command_timeout_max_ms: integer(
      motionTuning.command_timeout_max_ms,
      'motion_tuning.command_timeout_max_ms',
    ),
  };
  if (
    normalizedMotionTuning.gain_min < 0
    || normalizedMotionTuning.gain_max < normalizedMotionTuning.gain_min
    || normalizedMotionTuning.axis_max_output_min < 0
    || normalizedMotionTuning.axis_max_output_max
      < normalizedMotionTuning.axis_max_output_min
    || normalizedMotionTuning.global_multiplier_min < 0
    || normalizedMotionTuning.global_multiplier_max
      < normalizedMotionTuning.global_multiplier_min
    || normalizedMotionTuning.pwm_slew_rate_max_us_per_s
      < normalizedMotionTuning.pwm_slew_rate_min_us_per_s
    || normalizedMotionTuning.command_timeout_max_ms
      < normalizedMotionTuning.command_timeout_min_ms
  ) {
    throw new Error('运动调参能力范围无效');
  }
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
      motion_tuning: features.motion_tuning === true,
      sensor_stream: features.sensor_stream === true,
      emergency_stop: features.emergency_stop === true,
    },
    motion_tuning: normalizedMotionTuning,
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
