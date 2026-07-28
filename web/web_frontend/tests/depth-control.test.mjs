import assert from 'node:assert/strict';
import { Buffer } from 'node:buffer';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { transformSync } from 'esbuild';

const source = readFileSync(
  new URL('../src/depthControl.ts', import.meta.url),
  'utf8',
);
const { code } = transformSync(source, { loader: 'ts', format: 'esm' });
const depth = await import(
  `data:text/javascript;base64,${Buffer.from(code).toString('base64')}`
);

const tuning = {
  kp: 10,
  ki: 0.5,
  kd: 4,
  p_limit_us: 120,
  i_limit_us: 60,
  d_limit_us: 80,
  output_limit_us: 220,
};

function validSource(timestamp = 100) {
  return {
    status_stale: false,
    sensors_stale: false,
    depth_sensor_ready: true,
    depth_sample_valid: true,
    depth_sample_age_ms: 25,
    depth_m: 2.1,
    depth_active_setpoint_m: 2.5,
    depth_error_m: 0.4,
    depth_pid_output_us: 40,
    depth_pid_p_us: 30,
    depth_pid_i_us: 8,
    depth_pid_d_us: 2,
    last_depth_control_report_at: timestamp,
  };
}

test('depth tuning accepts wrapped readback and direct response', () => {
  const wrapped = depth.normalizeDepthTuningSnapshot({
    desired: tuning,
    confirmed: { ...tuning, kp: 9.5 },
    synced: false,
    sync_state: 'mismatch',
    sync_error: 'kp mismatch',
  });
  assert.equal(wrapped.desired.kp, 10);
  assert.equal(wrapped.confirmed.kp, 9.5);
  assert.equal(wrapped.synced, false);
  assert.equal(wrapped.sync_error, 'kp mismatch');

  const direct = depth.normalizeDepthTuningSnapshot(tuning);
  assert.deepEqual(direct.desired, tuning);
  assert.deepEqual(direct.confirmed, tuning);
  assert.equal(direct.synced, true);
});

test('depth tuning rejects missing and non-finite PID values', () => {
  assert.throws(
    () => depth.normalizeDepthTuningSnapshot({ ...tuning, kd: Infinity }),
    /PID/,
  );
  assert.throws(
    () => depth.normalizeDepthTuningSnapshot({ ...tuning, kd: null }),
    /PID/,
  );
  assert.throws(
    () => depth.normalizeDepthTuningSnapshot({
      desired: tuning,
      confirmed: { ...tuning, output_limit_us: 'bad' },
    }),
    /PID/,
  );
});

test('history admits only fresh, ready and finite depth control reports', () => {
  const point = depth.depthHistoryPoint(validSource());
  assert.equal(point.timestamp_s, 100);
  assert.equal(point.actual_m, 2.1);
  assert.equal(point.target_m, 2.5);

  for (const invalid of [
    { status_stale: true },
    { sensors_stale: true },
    { depth_sensor_ready: false },
    { depth_sample_valid: false },
    { depth_sample_age_ms: null },
    { depth_m: NaN },
    { depth_pid_output_us: Infinity },
    { last_depth_control_report_at: 0 },
  ]) {
    assert.equal(
      depth.depthHistoryPoint({ ...validSource(), ...invalid }),
      null,
    );
  }

  const milliseconds = depth.depthHistoryPoint(
    validSource(1_720_000_000_000),
  );
  assert.equal(milliseconds.timestamp_s, 1_720_000_000);
});

test('history keeps 120 seconds, replaces duplicates, and resets on time reversal', () => {
  const points = [0, 30, 121, 130]
    .map(timestamp => depth.depthHistoryPoint(validSource(timestamp + 1)));
  let history = [];
  for (const point of points) history = depth.appendDepthHistory(history, point);
  assert.deepEqual(
    history.map(point => point.timestamp_s),
    [31, 122, 131],
  );

  const replacement = {
    ...history.at(-1),
    actual_m: 3.25,
  };
  history = depth.appendDepthHistory(history, replacement);
  assert.equal(history.length, 3);
  assert.equal(history.at(-1).actual_m, 3.25);

  const reset = depth.depthHistoryPoint(validSource(5));
  history = depth.appendDepthHistory(history, reset);
  assert.deepEqual(history, [reset]);
});

test('SVG paths and chart ranges remain finite', () => {
  const history = [100, 110, 120].map(timestamp => (
    depth.depthHistoryPoint(validSource(timestamp))
  ));
  const range = depth.finiteChartRange(
    history.map(point => point.actual_m),
    0.2,
  );
  assert.equal(range.every(Number.isFinite), true);
  assert.ok(range[1] > range[0]);

  const path = depth.chartPath(
    history,
    'actual_m',
    0,
    120,
    range[0],
    range[1],
  );
  assert.match(path, /^M/);
  assert.doesNotMatch(path, /NaN|Infinity/);
});
