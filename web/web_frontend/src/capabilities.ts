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
  depth_hold: boolean;
  gamepad_control: boolean;
  sensor_stream: boolean;
  emergency_stop: boolean;
}

export interface DepthPidCapabilities {
  kp_min: number;
  kp_max: number;
  ki_min: number;
  ki_max: number;
  kd_min: number;
  kd_max: number;
  term_limit_min_us: number;
  term_limit_max_us: number;
  output_limit_min_us: number;
  output_limit_max_us: number;
  target_depth_min_m: number;
  target_depth_max_m: number;
  lease_timeout_ms: number;
}

export interface GamepadCapabilities {
  axis_count: number;
  min_button_count: number;
  max_button_count: number;
  send_hz: number;
  zero_timeout_ms: number;
  disconnect_timeout_ms: number;
  deadzone: number;
  expo: number;
  global_scale: number;
  surge_scale: number;
  sway_scale: number;
  heave_scale: number;
  yaw_scale: number;
  heave_button_strength: number;
  surge_invert: boolean;
  sway_invert: boolean;
  yaw_invert: boolean;
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
  depth_pid: DepthPidCapabilities;
  gamepad: GamepadCapabilities;
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
    depth_hold: false,
    gamepad_control: false,
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
  depth_pid: {
    kp_min: 0,
    kp_max: 0,
    ki_min: 0,
    ki_max: 0,
    kd_min: 0,
    kd_max: 0,
    term_limit_min_us: 0,
    term_limit_max_us: 0,
    output_limit_min_us: 0,
    output_limit_max_us: 0,
    target_depth_min_m: 0,
    target_depth_max_m: 0,
    lease_timeout_ms: 0,
  },
  gamepad: {
    axis_count: 6,
    min_button_count: 4,
    max_button_count: 4,
    send_hz: 50,
    zero_timeout_ms: 300,
    disconnect_timeout_ms: 1000,
    deadzone: 0.08,
    expo: 1,
    global_scale: 0,
    surge_scale: 0,
    sway_scale: 0,
    heave_scale: 0,
    yaw_scale: 0,
    heave_button_strength: 0,
    surge_invert: true,
    sway_invert: false,
    yaw_invert: false,
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

function normalizeGamepad(
  value: unknown,
  enabled: boolean,
): GamepadCapabilities {
  if (!isRecord(value)) {
    if (enabled) throw new Error('能力配置缺少 gamepad 对象');
    return { ...LOCKED_CAPABILITIES.gamepad };
  }
  const result: GamepadCapabilities = {
    axis_count: integer(value.axis_count, 'gamepad.axis_count'),
    min_button_count: integer(
      value.min_button_count, 'gamepad.min_button_count'),
    max_button_count: integer(
      value.max_button_count, 'gamepad.max_button_count'),
    send_hz: finiteNumber(value.send_hz, 'gamepad.send_hz'),
    zero_timeout_ms: integer(
      value.zero_timeout_ms, 'gamepad.zero_timeout_ms'),
    disconnect_timeout_ms: integer(
      value.disconnect_timeout_ms, 'gamepad.disconnect_timeout_ms'),
    deadzone: finiteNumber(value.deadzone, 'gamepad.deadzone'),
    expo: finiteNumber(value.expo, 'gamepad.expo'),
    global_scale: finiteNumber(value.global_scale, 'gamepad.global_scale'),
    surge_scale: finiteNumber(value.surge_scale, 'gamepad.surge_scale'),
    sway_scale: finiteNumber(value.sway_scale, 'gamepad.sway_scale'),
    heave_scale: finiteNumber(value.heave_scale, 'gamepad.heave_scale'),
    yaw_scale: finiteNumber(value.yaw_scale, 'gamepad.yaw_scale'),
    heave_button_strength: finiteNumber(
      value.heave_button_strength, 'gamepad.heave_button_strength'),
    surge_invert: value.surge_invert === true,
    sway_invert: value.sway_invert === true,
    yaw_invert: value.yaw_invert === true,
  };
  if (
    result.axis_count !== 6
    || result.min_button_count < 4
    || result.max_button_count < result.min_button_count
    || result.send_hz < 20
    || result.send_hz > 100
    || result.zero_timeout_ms < 100
    || result.disconnect_timeout_ms <= result.zero_timeout_ms
    || result.deadzone < 0
    || result.deadzone >= 0.5
    || result.expo < 1
    || result.expo > 3
    || [
      result.global_scale,
      result.surge_scale,
      result.sway_scale,
      result.heave_scale,
      result.yaw_scale,
      result.heave_button_strength,
    ].some(item => item < 0 || item > 1)
  ) {
    throw new Error('手柄能力配置范围无效');
  }
  return result;
}

function normalizeDepthPid(
  value: unknown,
  enabled: boolean,
): DepthPidCapabilities {
  if (!enabled) return { ...LOCKED_CAPABILITIES.depth_pid };
  if (!isRecord(value)) {
    throw new Error('能力配置缺少 depth_pid 对象');
  }
  const result: DepthPidCapabilities = {
    kp_min: finiteNumber(value.kp_min, 'depth_pid.kp_min'),
    kp_max: finiteNumber(value.kp_max, 'depth_pid.kp_max'),
    ki_min: finiteNumber(value.ki_min, 'depth_pid.ki_min'),
    ki_max: finiteNumber(value.ki_max, 'depth_pid.ki_max'),
    kd_min: finiteNumber(value.kd_min, 'depth_pid.kd_min'),
    kd_max: finiteNumber(value.kd_max, 'depth_pid.kd_max'),
    term_limit_min_us: finiteNumber(
      value.term_limit_min_us,
      'depth_pid.term_limit_min_us',
    ),
    term_limit_max_us: finiteNumber(
      value.term_limit_max_us,
      'depth_pid.term_limit_max_us',
    ),
    output_limit_min_us: finiteNumber(
      value.output_limit_min_us,
      'depth_pid.output_limit_min_us',
    ),
    output_limit_max_us: finiteNumber(
      value.output_limit_max_us,
      'depth_pid.output_limit_max_us',
    ),
    target_depth_min_m: finiteNumber(
      value.target_depth_min_m,
      'depth_pid.target_depth_min_m',
    ),
    target_depth_max_m: finiteNumber(
      value.target_depth_max_m,
      'depth_pid.target_depth_max_m',
    ),
    lease_timeout_ms: integer(
      value.lease_timeout_ms,
      'depth_pid.lease_timeout_ms',
    ),
  };
  if (
    result.kp_min < 0
    || result.kp_max < result.kp_min
    || result.ki_min < 0
    || result.ki_max < result.ki_min
    || result.kd_min < 0
    || result.kd_max < result.kd_min
    || result.term_limit_min_us < 0
    || result.term_limit_max_us < result.term_limit_min_us
    || result.output_limit_min_us < 0
    || result.output_limit_max_us < result.output_limit_min_us
    || result.target_depth_max_m < result.target_depth_min_m
    || result.lease_timeout_ms <= 200
  ) {
    throw new Error('定深 PID 能力配置范围无效');
  }
  return result;
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
  const gamepadEnabled = features.gamepad_control === true;
  const depthHoldEnabled = features.depth_hold === true;
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
      depth_hold: depthHoldEnabled,
      gamepad_control: gamepadEnabled,
      sensor_stream: features.sensor_stream === true,
      emergency_stop: features.emergency_stop === true,
    },
    motion_tuning: normalizedMotionTuning,
    depth_pid: normalizeDepthPid(raw.depth_pid, depthHoldEnabled),
    gamepad: normalizeGamepad(raw.gamepad, gamepadEnabled),
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
