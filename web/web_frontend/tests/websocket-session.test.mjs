import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { Buffer } from 'node:buffer';
import { transformSync } from 'esbuild';

const source = readFileSync(new URL('../src/wsProtocol.ts', import.meta.url), 'utf8');
const { code } = transformSync(source, { loader: 'ts', format: 'esm' });
const protocol = await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);
const { acceptTelemetryMessage, createTelemetryCursor } = protocol;

function message(sessionId, sequence, payload = {}) {
  return {
    type: 'status',
    session_id: sessionId,
    sequence,
    timestamp: 1_800_000_000 + sequence,
    payload,
  };
}

test('a backend session change accepts sequence 1 after sequence 500', () => {
  const first = acceptTelemetryMessage(message('session-A', 500, { marker: 'A' }), createTelemetryCursor());
  assert.equal(first.accepted, true);

  const restarted = acceptTelemetryMessage(message('session-B', 1, { marker: 'B' }), first.cursor);
  assert.equal(restarted.accepted, true);
  assert.equal(restarted.sessionChanged, true);
  assert.equal(restarted.payload.marker, 'B');
  assert.equal(restarted.cursor.sequence, 1);
});

test('same-session duplicate and older messages are rejected', () => {
  const first = acceptTelemetryMessage(message('session-A', 500), createTelemetryCursor());
  assert.equal(first.accepted, true);
  for (const sequence of [500, 499, 1]) {
    const decision = acceptTelemetryMessage(message('session-A', sequence), first.cursor);
    assert.equal(decision.accepted, false);
    assert.equal(decision.reason, 'out_of_order');
  }
});

test('a new socket cursor does not inherit the previous connection sequence', () => {
  const oldConnection = acceptTelemetryMessage(message('session-A', 500), createTelemetryCursor());
  assert.equal(oldConnection.accepted, true);

  const newConnection = acceptTelemetryMessage(message('session-A', 1), createTelemetryCursor());
  assert.equal(newConnection.accepted, true);
  assert.equal(newConnection.cursor.sequence, 1);
});

test('an unscoped message cannot replace a session-scoped stream', () => {
  const first = acceptTelemetryMessage(message('session-A', 7), createTelemetryCursor());
  assert.equal(first.accepted, true);

  const unscoped = acceptTelemetryMessage({ type: 'status', sequence: 8, payload: {} }, first.cursor);
  assert.equal(unscoped.accepted, false);
  assert.equal(unscoped.reason, 'missing_session');
});

test('sequence zero and uint32 wraparound are handled without truthiness bugs', () => {
  const last = acceptTelemetryMessage(message('session-A', 0xFFFFFFFF), createTelemetryCursor());
  assert.equal(last.accepted, true);
  const wrapped = acceptTelemetryMessage(message('session-A', 0), last.cursor);
  assert.equal(wrapped.accepted, true);
  assert.equal(wrapped.cursor.sequence, 0);
});

test('legacy timestamp ordering remains fail-safe when sequence is absent', () => {
  const first = acceptTelemetryMessage({
    type: 'status',
    timestamp: '2026-07-10T10:00:00Z',
    payload: { marker: 'new' },
  }, createTelemetryCursor());
  assert.equal(first.accepted, true);

  const older = acceptTelemetryMessage({
    type: 'status',
    timestamp: '2026-07-10T09:59:59Z',
    payload: { marker: 'old' },
  }, first.cursor);
  assert.equal(older.accepted, false);
  assert.equal(older.reason, 'out_of_order');
});
