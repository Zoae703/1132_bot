import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { Buffer } from 'node:buffer';
import { transformSync } from 'esbuild';

async function importTypeScript(relativePath) {
  const source = readFileSync(new URL(relativePath, import.meta.url), 'utf8');
  const { code } = transformSync(source, { loader: 'ts', format: 'esm' });
  return import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);
}

const { normalizeStatus } = await importTypeScript('../src/status.ts');

test('normalizeStatus uses fail-safe defaults for missing dangerous fields', () => {
  const status = normalizeStatus({});
  assert.equal(status.mode, 'UNKNOWN');
  assert.equal(status.safety_state, -1);
  assert.equal(status.safety_state_name, 'UNKNOWN_-1');
  assert.equal(status.serial_connected, false);
  assert.equal(status.stm32_online, false);
  assert.equal(status.status_stale, true);
  assert.equal(status.sensors_stale, true);
  assert.equal(status.estop_locked, true);
  assert.equal(status.body_control_enabled, false);
  assert.equal(status.motion_tuning_synced, false);
  assert.equal(status.control_mode, 'UNKNOWN');
  assert.equal(status.gamepad.client_connected, false);
  assert.equal(status.gamepad.eligibility_reason, 'no_gamepad_frame');
  assert.deepEqual(status.confirmed_pwm, []);
  assert.deepEqual(status.requested_pwm, []);
  assert.equal(Number.isNaN(status.depth_m), true);
});

test('normalizeStatus exposes validated gamepad telemetry fields', () => {
  const status = normalizeStatus({
    control_mode: 'GAMEPAD',
    gamepad: {
      client_connected: true,
      lease_session_id: 'session_1234',
      lease_active: true,
      gamepad_connected: true,
      control_enabled: true,
      eligible: true,
      eligibility_reason: 'accepted',
      axes: [-1, 0.5, 0, 0, 0.2, 0],
      buttons: [0, 0, 0, 1],
      mapped_command: {
        surge: 0.15,
        sway: 0.07,
        heave: -0.015,
        roll: 0,
        pitch: 0,
        yaw: 0.03,
      },
      heave_conflict: false,
      last_sequence: 42,
      command_age_ms: 18,
      last_forwarded_sequence: 42,
      last_stm32_ack: true,
      zero_timeout_ms: 300,
      disconnect_timeout_ms: 1000,
      send_hz: 50,
    },
  });
  assert.equal(status.control_mode, 'GAMEPAD');
  assert.equal(status.gamepad.client_connected, true);
  assert.deepEqual(status.gamepad.axes, [-1, 0.5, 0, 0, 0.2, 0]);
  assert.equal(status.gamepad.mapped_command.surge, 0.15);
  assert.equal(status.gamepad.mapped_command.heave, -0.015);
  assert.equal(status.gamepad.command_age_ms, 18);
  assert.equal(status.gamepad.last_stm32_ack, true);
});

test('normalizeStatus exposes six-axis and saturation state', () => {
  const status = normalizeStatus({
    safety_state: 2,
    estop_locked: false,
    body_control_enabled: true,
    horizontal_saturated: true,
    vertical_saturated: false,
    motion_tuning_synced: true,
    motion_tuning_sync_state: 'synced',
  });
  assert.equal(status.body_control_enabled, true);
  assert.equal(status.horizontal_saturated, true);
  assert.equal(status.vertical_saturated, false);
  assert.equal(status.motion_tuning_synced, true);
  assert.equal(status.motion_tuning_sync_state, 'synced');
});

test('normalizeStatus keeps requested and confirmed PWM distinct', () => {
  const status = normalizeStatus({
    mode: 'SIMULATION',
    serial_connected: true,
    stm32_online: true,
    status_stale: false,
    sensors_stale: false,
    estop_locked: false,
    safety_state: 3,
    requested_pwm: [1530, 1500],
    confirmed_pwm: [1500, 1500],
    request_state: 'pending',
    last_command_error: 'waiting for status',
  });
  assert.equal(status.safety_state_name, 'MANUAL_TEST');
  assert.deepEqual(status.requested_pwm, [1530, 1500]);
  assert.deepEqual(status.desired_pwm, [1530, 1500]);
  assert.deepEqual(status.confirmed_pwm, [1500, 1500]);
  assert.deepEqual(status.pwm, [1500, 1500]);
  assert.equal(status.request_state, 'pending');
  assert.equal(status.last_command_error, 'waiting for status');
});

test('normalizeStatus supports compatibility PWM names without inventing missing values', () => {
  const status = normalizeStatus({
    pwm: [1500, null, '1530'],
    desired_pwm: [1520],
  });
  assert.equal(status.confirmed_pwm[0], 1500);
  assert.equal(Number.isNaN(status.confirmed_pwm[1]), true);
  assert.equal(Number.isNaN(status.confirmed_pwm[2]), true);
  assert.deepEqual(status.requested_pwm, [1520]);
});

test('normalizeStatus is null, undefined, and non-finite safe', () => {
  for (const value of [null, undefined, NaN, Infinity, 'bad payload', []]) {
    assert.doesNotThrow(() => normalizeStatus(value));
  }
  const status = normalizeStatus({
    depth_m: NaN,
    status_age_ms: Infinity,
    sequence: NaN,
    accel: [null, undefined, Infinity],
  });
  assert.equal(Number.isNaN(status.depth_m), true);
  assert.equal(status.status_age_ms, null);
  assert.equal(status.sequence, undefined);
  assert.equal(status.accel.every(Number.isNaN), true);
});
