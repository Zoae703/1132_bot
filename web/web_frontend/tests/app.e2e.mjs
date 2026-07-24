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
