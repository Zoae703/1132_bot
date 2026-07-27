import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { createServer } from 'node:net';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright-core';

const repoRoot = fileURLToPath(new URL('../..', import.meta.url));
const chromeCandidates = [
  process.env.ROV_CHROME_PATH,
  '/usr/bin/google-chrome',
  '/usr/bin/chromium',
  '/usr/bin/chromium-browser',
].filter(Boolean);
const chromePath = chromeCandidates.find(candidate => existsSync(candidate));

if (!chromePath) {
  throw new Error(
    'No Chrome/Chromium binary found. Set ROV_CHROME_PATH to run browser E2E tests.',
  );
}

function freePort() {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.once('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      const port = typeof address === 'object' && address ? address.port : null;
      server.close(error => {
        if (error) reject(error);
        else resolve(port);
      });
    });
  });
}

function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function waitFor(predicate, timeoutMs = 8000, intervalMs = 100) {
  const deadline = Date.now() + timeoutMs;
  let lastError;
  while (Date.now() < deadline) {
    try {
      const value = await predicate();
      if (value) return value;
    } catch (error) {
      lastError = error;
    }
    await delay(intervalMs);
  }
  throw lastError ?? new Error('condition timed out');
}

async function waitForExit(child, timeoutMs) {
  if (child.exitCode !== null) return true;
  return new Promise(resolve => {
    const timeout = setTimeout(() => {
      child.off('exit', onExit);
      resolve(false);
    }, timeoutMs);
    const onExit = () => {
      clearTimeout(timeout);
      resolve(true);
    };
    child.once('exit', onExit);
  });
}

const port = await freePort();
assert.equal(typeof port, 'number');
const baseUrl = `http://127.0.0.1:${port}`;
let backend = null;
let backendOutput = '';

async function startBackend() {
  backendOutput = '';
  backend = spawn(
    'python3',
    [
      '-m', 'opi_console.main',
      '--simulate',
      '--web-host', '127.0.0.1',
      '--web-port', String(port),
      '--log-level', 'WARNING',
    ],
    {
      cwd: repoRoot,
      env: {
        ...process.env,
        PYTHONPATH: `${repoRoot}:${repoRoot}/protocol/shared`,
      },
      stdio: ['ignore', 'pipe', 'pipe'],
    },
  );
  backend.stdout.on('data', chunk => { backendOutput += chunk.toString(); });
  backend.stderr.on('data', chunk => { backendOutput += chunk.toString(); });

  await waitFor(async () => {
    if (backend.exitCode !== null) {
      throw new Error(`Backend exited during startup:\n${backendOutput}`);
    }
    const response = await fetch(`${baseUrl}/health`).catch(() => null);
    return response?.ok;
  }, 10000);
}

async function stopBackend() {
  if (!backend || backend.exitCode !== null) return;
  backend.kill('SIGINT');
  if (await waitForExit(backend, 8000)) return;
  backend.kill('SIGTERM');
  if (await waitForExit(backend, 3000)) return;
  backend.kill('SIGKILL');
  await waitForExit(backend, 2000);
}

let browser;
try {
  await startBackend();
  browser = await chromium.launch({
    executablePath: chromePath,
    headless: true,
    args: ['--no-sandbox', '--disable-dev-shm-usage'],
  });
  const context = await browser.newContext();
  const page = await context.newPage();
  const pageErrors = [];
  let capabilityRequests = 0;
  let pwmRequests = 0;

  page.on('pageerror', error => pageErrors.push(error.message));
  page.on('console', message => {
    if (message.type() === 'error') pageErrors.push(message.text());
  });
  page.on('request', request => {
    if (request.url().endsWith('/api/capabilities')) capabilityRequests += 1;
    if (request.method() === 'POST' && request.url().endsWith('/api/pwm/test')) {
      pwmRequests += 1;
    }
  });
  await page.addInitScript(() => {
    globalThis.confirm = () => true;
  });

  await page.goto(baseUrl, { waitUntil: 'domcontentloaded' });
  await page.getByText('WebSocket 已连接', { exact: true }).waitFor();
  await page.getByText('未解锁', { exact: true }).first().waitFor();
  await page.getByText('链路与状态数据正常，可以按当前安全状态执行操作', {
    exact: true,
  }).waitFor();
  assert.equal(await page.getByRole('button', { name: '解锁' }).isEnabled(), true);
  assert.equal(await page.getByRole('button', { name: '急停' }).isEnabled(), true);
  assert.equal(await page.locator('.output-cell').count(), 8);

  const failedArm = async route => {
    await route.fulfill({
      status: 503,
      contentType: 'application/json',
      body: JSON.stringify({ detail: 'STM32 offline test' }),
    });
  };
  await page.route('**/api/arm', failedArm);
  await page.getByRole('button', { name: '解锁' }).click();
  await page.getByText(/解锁 失败：STM32 offline test/).waitFor();
  await page.unroute('**/api/arm', failedArm);
  await waitFor(() => page.getByRole('button', { name: '解锁' }).isEnabled());

  await page.getByRole('button', { name: '解锁' }).click();
  await page.getByText('已解锁待机', { exact: true }).first().waitFor();
  await page.getByRole('button', { name: '进入手动测试' }).click();
  await page.getByText('手动测试', { exact: true }).first().waitFor();
  await page.locator('input[type="range"]').fill('1530');

  const firstPwmResponse = page.waitForResponse(response => (
    response.url().endsWith('/api/pwm/test')
    && response.request().method() === 'POST'
  ));
  await page.getByRole('button', { name: '输出 CH0' }).evaluate(button => {
    button.click();
    button.click();
  });
  assert.equal((await firstPwmResponse).status(), 200);
  await page.getByText('确认 1530us', { exact: true }).waitFor();
  assert.equal(pwmRequests, 1, 'synchronous double click submitted more than once');
  await page.locator('.output-cell').nth(0)
    .getByText('确认 1500us', { exact: true })
    .waitFor({ timeout: 3000 });

  await page.getByRole('button', { name: '上锁并回中' }).click();
  await page.getByText('未解锁', { exact: true }).first().waitFor();

  await page.getByRole('button', { name: '运动调参', exact: true }).click();
  await page.getByRole('heading', { name: '运动调参' }).waitFor();
  await page.getByRole('button', { name: '解锁' }).click();
  await page.getByText('已解锁待机', { exact: true }).first().waitFor();
  await page.getByRole('button', { name: '进入六轴控制' }).click();
  await page.getByText('六轴模式已开启', { exact: true }).waitFor();

  const motionCommandResponse = page.waitForResponse(response => (
    response.url().endsWith('/api/motion/command')
    && response.request().method() === 'POST'
  ));
  const motionStopResponse = page.waitForResponse(response => (
    response.url().endsWith('/api/motion/stop')
    && response.request().method() === 'POST'
  ));
  const forwardButton = page.getByRole('button', { name: /前进/ });
  await forwardButton.hover();
  await page.mouse.down();
  await delay(180);
  assert.equal((await motionCommandResponse).status(), 200);
  await page.mouse.up();
  assert.equal((await motionStopResponse).status(), 200);

  await waitFor(async () => {
    const response = await fetch(`${baseUrl}/api/status`);
    if (!response.ok) return false;
    const current = await response.json();
    return current.body_control_enabled
      && current.confirmed_pwm.every(value => value === 1500);
  }, 3000);
  await page.getByRole('button', { name: '停止并退出' }).click();
  await page.getByText('已解锁待机', { exact: true }).first().waitFor();
  await page.getByRole('button', { name: '上锁并回中' }).click();
  await page.getByText('未解锁', { exact: true }).first().waitFor();

  await page.getByRole('button', { name: '手柄控制', exact: true }).click();
  const gamepadPanel = page.locator('section.panel').filter({
    has: page.getByRole('heading', { name: '手柄控制' }),
  });
  await gamepadPanel.waitFor();
  await page.evaluate(() => new Promise((resolve, reject) => {
    const socket = new WebSocket(
      `ws://${globalThis.location.host}/ws/control/gamepad`,
    );
    const state = {
      socket,
      config: null,
      sequence: 0,
      axes: [0, 0, 0, 0, 0, 0],
      buttons: [0, 0, 0, 0],
      timer: null,
      lastAck: null,
      send: null,
    };
    globalThis.__syntheticGamepad = state;
    const shape = (raw, invert) => {
      const magnitude = Math.abs(raw);
      if (magnitude <= state.config.deadzone) return 0;
      const normalized = (
        (magnitude - state.config.deadzone)
        / (1 - state.config.deadzone)
      );
      const value = Math.sign(raw) * normalized ** state.config.expo;
      return invert ? -value : value;
    };
    state.send = () => {
      if (socket.readyState !== WebSocket.OPEN || !state.config) return;
      const surge = shape(
        state.axes[1], state.config.surge_invert,
      ) * state.config.surge_scale * state.config.global_scale;
      const sway = shape(
        state.axes[0], state.config.sway_invert,
      ) * state.config.sway_scale * state.config.global_scale;
      const yaw = shape(
        state.axes[4], state.config.yaw_invert,
      ) * state.config.yaw_scale * state.config.global_scale;
      const a = state.buttons[0] === 1;
      const y = state.buttons[3] === 1;
      const heave = a === y ? 0 : (
        (a ? 1 : -1)
        * state.config.heave_button_strength
        * state.config.heave_scale
        * state.config.global_scale
      );
      state.sequence += 1;
      socket.send(JSON.stringify({
        type: 'gamepad_state',
        version: 1,
        session_id: 'browser_gamepad_e2e',
        sequence: state.sequence,
        client_time_ns: Math.trunc(performance.now() * 1_000_000),
        control_enabled: true,
        gamepad_connected: true,
        axes: state.axes,
        buttons: state.buttons,
        hats: [[0, 0]],
        device: { name: 'Browser synthetic gamepad' },
        mapped_command: {
          surge,
          sway,
          heave,
          roll: 0,
          pitch: 0,
          yaw,
        },
      }));
    };
    socket.onerror = () => reject(new Error('synthetic gamepad failed'));
    socket.onmessage = event => {
      const message = JSON.parse(event.data);
      if (message.type === 'gamepad_hello') {
        state.config = message.mapping_config;
        state.send();
        state.timer = setInterval(state.send, 25);
        resolve();
      } else if (message.type === 'gamepad_ack') {
        state.lastAck = message;
      }
    };
  }));

  await gamepadPanel.locator('.metric').filter({
    hasText: '电脑转发程序',
  }).getByText('已连接', { exact: true }).waitFor();
  await gamepadPanel.locator('.metric').filter({
    hasText: 'USB 手柄',
  }).getByText('已连接', { exact: true }).waitFor();
  await page.getByRole('button', { name: '解锁' }).click();
  await gamepadPanel.getByText('可进入 GAMEPAD 模式', { exact: true }).waitFor();
  await page.getByRole('button', { name: '进入 GAMEPAD' }).click();
  await page.getByText('控制模式 GAMEPAD', { exact: true }).waitFor();

  await page.evaluate(() => {
    globalThis.__syntheticGamepad.axes[1] = -1;
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    return status.control_mode === 'GAMEPAD'
      && status.gamepad.mapped_command.surge > 0
      && status.confirmed_pwm.some(value => value !== 1500);
  }, 4000);
  await gamepadPanel.getByText('-1.000', { exact: true }).waitFor();

  await page.evaluate(() => {
    clearInterval(globalThis.__syntheticGamepad.timer);
    globalThis.__syntheticGamepad.timer = null;
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    return status.control_mode === 'GAMEPAD'
      && status.gamepad.resume_requires_neutral
      && status.confirmed_pwm.every(value => value === 1500);
  }, 800);

  await page.evaluate(() => {
    const state = globalThis.__syntheticGamepad;
    state.send();
    state.timer = setInterval(state.send, 25);
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    return status.gamepad.command_age_ms < 300
      && status.gamepad.resume_requires_neutral
      && status.confirmed_pwm.every(value => value === 1500);
  }, 600);

  await page.evaluate(() => {
    globalThis.__syntheticGamepad.axes[1] = 0;
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    return !status.gamepad.resume_requires_neutral;
  }, 600);

  await page.evaluate(() => {
    globalThis.__syntheticGamepad.buttons[0] = 1;
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    const vertical = [1, 2, 5, 6].map(
      channel => status.confirmed_pwm[channel],
    );
    return status.gamepad.mapped_command.heave > 0
      && vertical.every(value => value > 1500)
      && new Set(vertical).size === 1;
  }, 4000);

  await page.evaluate(() => {
    const state = globalThis.__syntheticGamepad;
    state.buttons[0] = 0;
    state.buttons[3] = 1;
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    const vertical = [1, 2, 5, 6].map(
      channel => status.confirmed_pwm[channel],
    );
    return status.gamepad.mapped_command.heave < 0
      && vertical.every(value => value < 1500)
      && new Set(vertical).size === 1;
  }, 4000);

  await page.evaluate(() => {
    const state = globalThis.__syntheticGamepad;
    state.buttons[3] = 0;
    state.axes[4] = 1;
  });
  await waitFor(async () => {
    const status = await (await fetch(`${baseUrl}/api/status`)).json();
    return status.gamepad.mapped_command.yaw > 0
      && [0, 3, 4, 7].every(
        channel => status.confirmed_pwm[channel] > 1500,
      );
  }, 4000);

  await page.getByRole('button', { name: '退出并上锁' }).click();
  await page.getByText('未解锁', { exact: true }).first().waitFor();
  await page.evaluate(() => {
    const state = globalThis.__syntheticGamepad;
    if (state.timer) clearInterval(state.timer);
    state.socket.close();
  });

  await stopBackend();
  await page.getByText('WebSocket 断开', { exact: true }).waitFor({ timeout: 8000 });
  await page.getByText('WebSocket 已断开，页面离线，所有控制已锁定', {
    exact: true,
  }).waitFor();

  await startBackend();
  await page.getByText('WebSocket 已连接', { exact: true }).waitFor({ timeout: 10000 });
  await page.getByText('未解锁', { exact: true }).first().waitFor();
  await waitFor(() => capabilityRequests >= 2, 5000);
  await page.getByText(/检测到后端会话已重启/).waitFor();

  const delayedArm = async route => {
    await delay(1200);
    try {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ status: 'ok' }),
      });
    } catch {
      // ESTOP intentionally aborts the ordinary request in the page.
    }
  };
  await page.route('**/api/arm', delayedArm);
  await page.getByRole('button', { name: '解锁' }).click();
  assert.equal(
    await page.getByRole('button', { name: '急停' }).isEnabled(),
    true,
    'ESTOP was disabled by an ordinary in-flight request',
  );
  await page.getByRole('button', { name: '急停' }).click();
  await page.getByText('急停锁定', { exact: true }).first().waitFor({ timeout: 5000 });
  await page.unroute('**/api/arm', delayedArm);
  await page.getByRole('button', { name: '解除急停' }).click();
  await page.getByText('未解锁', { exact: true }).first().waitFor();

  await page.getByRole('button', { name: '状态与 PWM', exact: true }).click();
  await page.getByRole('button', { name: '解锁' }).click();
  await page.getByText('已解锁待机', { exact: true }).first().waitFor();
  await page.getByRole('button', { name: '进入手动测试' }).click();
  await page.getByText('手动测试', { exact: true }).first().waitFor();
  await page.getByRole('button', { name: /CH1\s+1500us/ }).click();
  await page.locator('input[type="range"]').fill('1530');
  await page.getByRole('button', { name: '2000ms' }).click();
  await page.getByRole('button', { name: '输出 CH1' }).click();
  await page.getByText('确认 1530us', { exact: true }).waitFor();
  await page.close();

  const finalStatus = await waitFor(async () => {
    const response = await fetch(`${baseUrl}/api/status`);
    if (!response.ok) return null;
    const status = await response.json();
    return status.safety_state_name === 'DISARMED'
      && status.confirmed_pwm.every(value => value === 1500)
      ? status
      : null;
  }, 5000);
  assert.deepEqual(finalStatus.confirmed_pwm, Array(8).fill(1500));

  const diagnostics = await (await fetch(`${baseUrl}/api/diagnostics`)).json();
  assert.equal(diagnostics.websocket_client_count, 0);
  assert.deepEqual(diagnostics.control.last_disconnect_result, {
    neutral: true,
    exit_manual: true,
    disarm: true,
  });

  const unexpectedErrors = pageErrors.filter(message => !(
    message.includes('WebSocket connection')
    || message.includes('ERR_CONNECTION_REFUSED')
    || message.includes('Failed to load resource')
  ));
  assert.deepEqual(unexpectedErrors, []);
  process.stdout.write(
    `browser E2E passed: capabilities=${capabilityRequests} pwm_posts=${pwmRequests}\n`,
  );
} finally {
  if (browser) await browser.close();
  await stopBackend();
}
