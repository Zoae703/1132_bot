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
      sensor_stream: true,
      emergency_stop: true,
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
});
