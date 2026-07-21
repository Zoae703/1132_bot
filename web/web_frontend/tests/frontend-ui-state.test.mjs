import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { Buffer } from 'node:buffer';
import { transformSync } from 'esbuild';

const source = readFileSync(new URL('../src/uiState.ts', import.meta.url), 'utf8');
const { code } = transformSync(source, { loader: 'ts', format: 'esm' });
const ui = await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);
const {
  apiErrorMessage,
  pwmControlAvailability,
  pwmDisplayPair,
  readinessView,
  sensorsFreshnessMessage,
  shouldClearLocalRequest,
} = ui;

const freshManual = {
  serial_connected: true,
  stm32_online: true,
  status_stale: false,
  safety_state: 3,
  estop_locked: false,
  backend_motion_inhibited: false,
};

test('WebSocket offline has priority over stale and hardware status', () => {
  const view = readinessView(false, { ...freshManual, status_stale: true }, true);
  assert.equal(view.tone, 'danger');
  assert.match(view.text, /WebSocket.*离线/);
});

test('STM32 offline and online-but-stale have distinct readiness messages', () => {
  const offline = readinessView(true, { ...freshManual, stm32_online: false }, true);
  const stale = readinessView(true, { ...freshManual, status_stale: true }, true);
  assert.match(offline.text, /STM32 离线/);
  assert.match(stale.text, /STM32 在线.*状态数据已过期/);
  assert.notEqual(offline.text, stale.text);
});

test('sensor stale is distinguished from WebSocket and STM32 offline', () => {
  assert.match(sensorsFreshnessMessage(false, { stm32_online: true, sensors_stale: false }), /WebSocket/);
  assert.match(sensorsFreshnessMessage(true, { stm32_online: false, sensors_stale: true }), /STM32 已离线/);
  assert.match(sensorsFreshnessMessage(true, { stm32_online: true, sensors_stale: true }), /传感器数据已过期/);
  assert.equal(sensorsFreshnessMessage(true, { stm32_online: true, sensors_stale: false }), null);
});

test('requested and confirmed PWM remain separate display values', () => {
  assert.deepEqual(pwmDisplayPair([1500], [1530], 0), { confirmed: 1500, requested: 1530 });
});

test('ACK timeout and NACK reasons remain explicit', () => {
  assert.equal(apiErrorMessage('ACK timeout for sequence 42', 'failed'), 'ACK timeout for sequence 42');
  assert.equal(apiErrorMessage('NACK INVALID_PWM for sequence 42', 'failed'), 'NACK INVALID_PWM for sequence 42');
  assert.equal(apiErrorMessage([{ msg: 'PWM must be safe' }], 'failed'), 'PWM must be safe');
});

test('ESTOP and local safety latch disable PWM but not neutral', () => {
  const estop = pwmControlAvailability({
    connected: true,
    status: { ...freshManual, safety_state: 5, estop_locked: true },
    capabilitiesReady: true,
    safetyLocked: true,
    requestPending: false,
  });
  assert.equal(estop.canTest, false);
  assert.equal(estop.canNeutral, true);

  const locallyLatched = pwmControlAvailability({
    connected: true,
    status: freshManual,
    capabilitiesReady: true,
    safetyLocked: true,
    requestPending: false,
  });
  assert.equal(locallyLatched.canTest, false);
});

test('backend uncertainty latch blocks motion but preserves neutral', () => {
  const inhibited = {
    ...freshManual,
    backend_motion_inhibited: true,
    backend_motion_inhibit_reason: 'emergency_stop_unconfirmed',
  };
  const view = readinessView(true, inhibited, true);
  assert.equal(view.tone, 'danger');
  assert.match(view.text, /后端运动锁定/);

  const availability = pwmControlAvailability({
    connected: true,
    status: inhibited,
    capabilitiesReady: true,
    safetyLocked: false,
    requestPending: false,
  });
  assert.equal(availability.canTest, false);
  assert.equal(availability.canNeutral, true);
});

test('MANUAL_TEST can send its first PWM before manual_pwm_enabled becomes true', () => {
  const availability = pwmControlAvailability({
    connected: true,
    status: freshManual,
    capabilitiesReady: true,
    safetyLocked: false,
    requestPending: false,
  });
  assert.equal(availability.canTest, true);
});

test('disconnect and safety lock clear local request state', () => {
  assert.equal(shouldClearLocalRequest(false, false), true);
  assert.equal(shouldClearLocalRequest(true, true), true);
  assert.equal(shouldClearLocalRequest(true, false), false);
});

test('missing capabilities lock motion but preserve neutral safety action', () => {
  const availability = pwmControlAvailability({
    connected: true,
    status: freshManual,
    capabilitiesReady: false,
    safetyLocked: false,
    requestPending: false,
  });
  assert.equal(availability.canTest, false);
  assert.equal(availability.canNeutral, true);
});
