import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { Buffer } from 'node:buffer';
import { transformSync } from 'esbuild';

const source = readFileSync(new URL('../src/capabilities.ts', import.meta.url), 'utf8');
const { code } = transformSync(source, { loader: 'ts', format: 'esm' });
const { LOCKED_CAPABILITIES, normalizeCapabilities } = await import(
  `data:text/javascript;base64,${Buffer.from(code).toString('base64')}`
);

function validCapabilities() {
  return {
    protocol_version: 2,
    channel_count: 8,
    pwm: {
      neutral_us: 1500,
      min_test_us: 1450,
      max_test_us: 1550,
      min_absolute_us: 1300,
      max_absolute_us: 1700,
      min_test_duration_ms: 200,
      max_test_duration_ms: 2000,
      default_timeout_ms: 500,
    },
    features: {
      manual_pwm: true,
      motor_mapping: true,
      motion_tuning: true,
      sensor_stream: true,
      emergency_stop: true,
    },
    motion_tuning: {
      axis_order: ['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw'],
      gain_min: 0,
      gain_max: 2,
      axis_max_output_min: 0,
      axis_max_output_max: 1,
      global_multiplier_min: 0,
      global_multiplier_max: 1,
      pwm_slew_rate_min_us_per_s: 100,
      pwm_slew_rate_max_us_per_s: 5000,
      command_timeout_min_ms: 200,
      command_timeout_max_ms: 2000,
    },
    telemetry: {
      status_hz: 5,
      sensors_hz: 5,
      status_stale_timeout_s: 2,
      sensors_stale_timeout_s: 2,
    },
    sensor_poll_hz: 5,
  };
}

test('locked capabilities cannot command movement', () => {
  assert.equal(LOCKED_CAPABILITIES.pwm.neutral_us, 1500);
  assert.equal(LOCKED_CAPABILITIES.pwm.min_test_us, 1500);
  assert.equal(LOCKED_CAPABILITIES.pwm.max_test_us, 1500);
  assert.equal(LOCKED_CAPABILITIES.features.motion_tuning, false);
});

test('valid backend capabilities are normalized', () => {
  assert.deepEqual(normalizeCapabilities(validCapabilities()), validCapabilities());
});

test('invalid or missing capabilities fail closed', () => {
  for (const value of [null, {}, { pwm: {} }]) {
    assert.throws(() => normalizeCapabilities(value));
  }
});

test('neutral, PWM bounds, and duration invariants are enforced', () => {
  const wrongNeutral = validCapabilities();
  wrongNeutral.pwm.neutral_us = 1490;
  assert.throws(() => normalizeCapabilities(wrongNeutral), /1500/);

  const inverted = validCapabilities();
  inverted.pwm.min_test_us = 1510;
  assert.throws(() => normalizeCapabilities(inverted), /范围/);

  const excessiveDefault = validCapabilities();
  excessiveDefault.pwm.default_timeout_ms = 2500;
  assert.throws(() => normalizeCapabilities(excessiveDefault), /默认测试时长/);

  const wrongProtocol = validCapabilities();
  wrongProtocol.protocol_version = 1;
  assert.throws(() => normalizeCapabilities(wrongProtocol), /protocol_version/);

  const invalidPollRate = validCapabilities();
  invalidPollRate.sensor_poll_hz = 0;
  assert.throws(() => normalizeCapabilities(invalidPollRate), /sensor_poll_hz/);

  const invalidTuning = validCapabilities();
  invalidTuning.motion_tuning.command_timeout_min_ms = 2500;
  assert.throws(() => normalizeCapabilities(invalidTuning), /运动调参/);
});
