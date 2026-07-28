import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';

const app = readFileSync(
  new URL('../src/App.tsx', import.meta.url),
  'utf8',
);
const panel = readFileSync(
  new URL('../src/DepthControlPanel.tsx', import.meta.url),
  'utf8',
);
const css = readFileSync(
  new URL('../src/App.css', import.meta.url),
  'utf8',
);

test('App exposes the depth tuning page', () => {
  assert.match(app, /import DepthControlPanel/);
  assert.match(app, /activePage === 'depth'/);
  assert.match(app, /定深调试/);
  assert.match(app, /telemetryVersion=\{telemetryVersion\}/);
  assert.match(app, /connectionVersion=\{connectionVersion\}/);
});

test('depth panel covers the complete backend API contract', () => {
  for (const route of [
    '/api/depth/tuning',
    '/api/depth/control',
    '/api/depth/enable',
    '/api/depth/target',
    '/api/depth/keepalive',
    '/api/depth/disable',
  ]) {
    assert.match(panel, new RegExp(route.replaceAll('/', '\\/')));
  }
  assert.match(panel, /KEEPALIVE_INTERVAL_MS = 200/);
  assert.match(panel, /window\.setInterval/);
  assert.match(panel, /haltKeepalive\(warning\)/);
  assert.match(panel, /target_depth_m: leaseTargetRef\.current/);
});

test('depth panel includes seven PID fields and two dependency-free SVG charts', () => {
  for (const field of [
    'kp',
    'ki',
    'kd',
    'p_limit_us',
    'i_limit_us',
    'd_limit_us',
    'output_limit_us',
  ]) {
    assert.match(panel, new RegExp(`key: '${field}'`));
  }
  assert.equal((panel.match(/<svg/g) ?? []).length, 2);
  assert.match(panel, /实际深度 \/ 目标深度/);
  assert.match(panel, /误差 \/ PID 输出/);
  assert.match(panel, /最近 120 秒/);
  assert.match(panel, /DISARMED \/ ARMED_IDLE/);
  assert.match(panel, /unit: 'us\/cm'/);
  assert.match(panel, /unit: 'us\/\(cm·采样\)'/);
  assert.match(panel, /unit: 'us·采样\/cm'/);
  assert.match(css, /\.depth-chart__line--actual/);
  assert.match(css, /\.depth-chart__line--output/);
});
