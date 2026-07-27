export const SAFETY_STATE_NAMES: Record<number, string> = {
  0: 'DISARMED',
  1: 'ARMED_IDLE',
  2: 'ARMED_ACTIVE',
  3: 'MANUAL_TEST',
  4: 'COMM_LOST',
  5: 'EMERGENCY_STOP',
  6: 'FAULT',
};

export const SAFETY_STATE_LABELS: Record<number, string> = {
  0: '未解锁',
  1: '已解锁待机',
  2: '自动控制中',
  3: '手动测试',
  4: '通信丢失',
  5: '急停锁定',
  6: '故障',
};

export type RequestState = 'idle' | 'pending' | 'confirmed' | 'rejected' | 'timeout' | 'unknown';

export interface GamepadCommand {
  surge: number;
  sway: number;
  heave: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export interface GamepadStatus {
  client_connected: boolean;
  lease_session_id: string | null;
  lease_active: boolean;
  gamepad_connected: boolean;
  control_enabled: boolean;
  eligible: boolean;
  eligibility_reason: string;
  axes: number[];
  buttons: number[];
  mapped_command: GamepadCommand;
  heave_conflict: boolean;
  last_sequence: number | null;
  command_age_ms: number | null;
  last_forwarded_sequence: number | null;
  last_stm32_ack: boolean;
  last_error: string | null;
  last_disconnect_reason: string | null;
  rejected_messages: number;
  duplicate_messages: number;
  resume_requires_neutral: boolean;
  zero_timeout_ms: number;
  disconnect_timeout_ms: number;
  send_hz: number;
}

export interface RobotStatus {
  mode: string;
  serial_connected: boolean;
  stm32_online: boolean;
  safety_state: number;
  safety_state_name: string;
  safety_state_label: string;
  control_enable: boolean;
  float_enabled: boolean;
  angle_enabled: boolean;
  manual_pwm_enabled: boolean;
  body_control_enabled: boolean;
  horizontal_saturated: boolean;
  vertical_saturated: boolean;
  motion_tuning_synced: boolean;
  motion_tuning_sync_state: string;
  motion_tuning_sync_error: string | null;
  estop_locked: boolean;
  backend_motion_inhibited: boolean;
  backend_motion_inhibit_reason: string | null;
  control_mode: string;
  gamepad: GamepadStatus;
  pwm: number[];
  confirmed_pwm: number[];
  requested_pwm: number[];
  /** Compatibility alias for requested_pwm during the backend rollout. */
  desired_pwm: number[];
  request_state: RequestState;
  last_command_error: string | null;
  neutral_reason: string | null;
  error_count: number;
  heartbeat_missed: number;
  tx_frames: number;
  rx_frames: number;
  rx_errors: number;
  crc_errors: number;
  ack_timeouts: number;
  nack_count: number;
  last_update: number;
  depth_m: number;
  pressure_mbar: number;
  water_temp_c: number;
  yaw: number;
  pitch: number;
  roll: number;
  accel: number[];
  gyro: number[];
  mag: number[];
  timestamp: number;
  sequence?: number;
  session_id?: string;
  status_stale: boolean;
  sensors_stale: boolean;
  status_age_ms: number | null;
  sensors_age_ms: number | null;
}

/** Layout fallback only. Missing telemetry is never normalized to these values. */
export const NEUTRAL_PWM = Array(8).fill(1500) as number[];

export function asNumber(value: unknown, fallback = 0): number {
  return typeof value === 'number' && Number.isFinite(value) ? value : fallback;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function asBool(value: unknown, fallback = false): boolean {
  return typeof value === 'boolean' ? value : fallback;
}

function asOptionalString(value: unknown): string | null {
  return typeof value === 'string' && value.trim() ? value : null;
}

function normalizePwm(value: unknown): number[] {
  if (!Array.isArray(value)) return [];
  return value.map(item => asNumber(item, NaN));
}

function pwmSource(primary: unknown, compatibility: unknown): number[] {
  if (Array.isArray(primary)) return normalizePwm(primary);
  return normalizePwm(compatibility);
}

function vector3(value: unknown): number[] {
  if (!Array.isArray(value)) return [NaN, NaN, NaN];
  return [0, 1, 2].map(index => asNumber(value[index], NaN));
}

function finiteVector(value: unknown): number[] {
  if (!Array.isArray(value)) return [];
  return value.map(item => asNumber(item, NaN));
}

function nullableNumber(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

function normalizeGamepadCommand(value: unknown): GamepadCommand {
  const source = isRecord(value) ? value : {};
  return {
    surge: asNumber(source.surge),
    sway: asNumber(source.sway),
    heave: asNumber(source.heave),
    roll: asNumber(source.roll),
    pitch: asNumber(source.pitch),
    yaw: asNumber(source.yaw),
  };
}

function normalizeGamepad(value: unknown): GamepadStatus {
  const source = isRecord(value) ? value : {};
  return {
    client_connected: asBool(source.client_connected),
    lease_session_id: asOptionalString(source.lease_session_id),
    lease_active: asBool(source.lease_active),
    gamepad_connected: asBool(source.gamepad_connected),
    control_enabled: asBool(source.control_enabled),
    eligible: asBool(source.eligible),
    eligibility_reason: asOptionalString(source.eligibility_reason)
      ?? 'no_gamepad_frame',
    axes: finiteVector(source.axes),
    buttons: finiteVector(source.buttons),
    mapped_command: normalizeGamepadCommand(source.mapped_command),
    heave_conflict: asBool(source.heave_conflict),
    last_sequence: nullableNumber(source.last_sequence),
    command_age_ms: nullableNumber(source.command_age_ms),
    last_forwarded_sequence: nullableNumber(source.last_forwarded_sequence),
    last_stm32_ack: asBool(source.last_stm32_ack),
    last_error: asOptionalString(source.last_error),
    last_disconnect_reason: asOptionalString(source.last_disconnect_reason),
    rejected_messages: asNumber(source.rejected_messages),
    duplicate_messages: asNumber(source.duplicate_messages),
    resume_requires_neutral: asBool(source.resume_requires_neutral),
    zero_timeout_ms: asNumber(source.zero_timeout_ms, 300),
    disconnect_timeout_ms: asNumber(source.disconnect_timeout_ms, 1000),
    send_hz: asNumber(source.send_hz, 50),
  };
}

function normalizeRequestState(value: unknown): RequestState {
  switch (value) {
    case 'idle':
    case 'pending':
    case 'confirmed':
    case 'rejected':
    case 'timeout':
      return value;
    default:
      return 'unknown';
  }
}

export function normalizeStatus(raw: unknown): RobotStatus {
  const source = isRecord(raw) ? raw : {};
  const safetyState = asNumber(source.safety_state, -1);
  const confirmedPwm = pwmSource(source.confirmed_pwm, source.pwm);
  const requestedPwm = pwmSource(source.requested_pwm, source.desired_pwm);
  const sequence = asNumber(source.sequence, NaN);
  const sessionId = asOptionalString(source.session_id);

  return {
    mode: asOptionalString(source.mode) ?? 'UNKNOWN',
    serial_connected: asBool(source.serial_connected),
    stm32_online: asBool(source.stm32_online),
    safety_state: safetyState,
    safety_state_name: asOptionalString(source.safety_state_name)
      ?? SAFETY_STATE_NAMES[safetyState]
      ?? `UNKNOWN_${safetyState}`,
    safety_state_label: asOptionalString(source.safety_state_label)
      ?? SAFETY_STATE_LABELS[safetyState]
      ?? `未知状态 ${safetyState}`,
    control_enable: asBool(source.control_enable),
    float_enabled: asBool(source.float_enabled),
    angle_enabled: asBool(source.angle_enabled),
    manual_pwm_enabled: asBool(source.manual_pwm_enabled),
    body_control_enabled: asBool(source.body_control_enabled),
    horizontal_saturated: asBool(source.horizontal_saturated),
    vertical_saturated: asBool(source.vertical_saturated),
    motion_tuning_synced: asBool(source.motion_tuning_synced),
    motion_tuning_sync_state: asOptionalString(
      source.motion_tuning_sync_state) ?? 'pending',
    motion_tuning_sync_error: asOptionalString(
      source.motion_tuning_sync_error),
    // Missing emergency-stop state is unsafe: keep motion controls locked.
    estop_locked: safetyState === 5 || asBool(source.estop_locked, true),
    backend_motion_inhibited: asBool(source.backend_motion_inhibited, true),
    backend_motion_inhibit_reason: asOptionalString(
      source.backend_motion_inhibit_reason),
    control_mode: asOptionalString(source.control_mode) ?? 'UNKNOWN',
    gamepad: normalizeGamepad(source.gamepad),
    pwm: confirmedPwm,
    confirmed_pwm: confirmedPwm,
    requested_pwm: requestedPwm,
    desired_pwm: requestedPwm,
    request_state: normalizeRequestState(source.request_state),
    last_command_error: asOptionalString(source.last_command_error),
    neutral_reason: asOptionalString(source.neutral_reason_name ?? source.neutral_reason),
    error_count: asNumber(source.error_count, 0),
    heartbeat_missed: asNumber(source.heartbeat_missed, 0),
    tx_frames: asNumber(source.tx_frames, 0),
    rx_frames: asNumber(source.rx_frames, 0),
    rx_errors: asNumber(source.rx_errors, 0),
    crc_errors: asNumber(source.crc_errors, 0),
    ack_timeouts: asNumber(source.ack_timeouts, 0),
    nack_count: asNumber(source.nack_count, 0),
    last_update: asNumber(source.last_update, 0),
    // Freshness is fail-safe: absent, null, or malformed means stale.
    status_stale: asBool(source.status_stale, true),
    sensors_stale: asBool(source.sensors_stale, true),
    status_age_ms: Number.isFinite(source.status_age_ms) ? Number(source.status_age_ms) : null,
    sensors_age_ms: Number.isFinite(source.sensors_age_ms) ? Number(source.sensors_age_ms) : null,
    depth_m: asNumber(source.depth_m, NaN),
    pressure_mbar: asNumber(source.pressure_mbar, NaN),
    water_temp_c: asNumber(source.water_temp_c, NaN),
    yaw: asNumber(source.yaw, NaN),
    pitch: asNumber(source.pitch, NaN),
    roll: asNumber(source.roll, NaN),
    accel: vector3(source.accel),
    gyro: vector3(source.gyro),
    mag: vector3(source.mag),
    timestamp: asNumber(source.timestamp, 0),
    sequence: Number.isFinite(sequence) ? sequence : undefined,
    session_id: sessionId ?? undefined,
  };
}
