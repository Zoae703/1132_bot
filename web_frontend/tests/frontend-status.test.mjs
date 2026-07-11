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
  assert.deepEqual(status.confirmed_pwm, []);
  assert.deepEqual(status.requested_pwm, []);
  assert.equal(Number.isNaN(status.depth_m), true);
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
