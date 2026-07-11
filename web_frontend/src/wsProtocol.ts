export interface TelemetryCursor {
  sessionId: string | null;
  sequence: number | null;
  timestamp: number | null;
}

export interface AcceptedTelemetry {
  accepted: true;
  cursor: TelemetryCursor;
  payload: Record<string, unknown>;
  sessionChanged: boolean;
}

export interface RejectedTelemetry {
  accepted: false;
  cursor: TelemetryCursor;
  reason: 'invalid_message' | 'unsupported_type' | 'missing_session' | 'out_of_order';
}

export type TelemetryDecision = AcceptedTelemetry | RejectedTelemetry;

export function createTelemetryCursor(): TelemetryCursor {
  return { sessionId: null, sequence: null, timestamp: null };
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function sessionId(value: unknown): string | null {
  return typeof value === 'string' && value.trim() ? value : null;
}

function sequenceNumber(value: unknown): number | null {
  return typeof value === 'number'
    && Number.isSafeInteger(value)
    && value >= 0
    && value <= 0xFFFFFFFF
    ? value
    : null;
}

function timestampNumber(value: unknown): number | null {
  if (typeof value === 'number' && Number.isFinite(value)) return value;
  if (typeof value === 'string' && value.trim()) {
    const parsed = Date.parse(value);
    return Number.isFinite(parsed) ? parsed / 1000 : null;
  }
  return null;
}

function isNewerSequence(next: number, previous: number): boolean {
  const delta = (next - previous) >>> 0;
  return delta !== 0 && delta < 0x80000000;
}

export function acceptTelemetryMessage(
  raw: unknown,
  cursor: TelemetryCursor,
): TelemetryDecision {
  if (!isRecord(raw)) {
    return { accepted: false, cursor, reason: 'invalid_message' };
  }
  if (raw.type !== 'status') {
    return { accepted: false, cursor, reason: 'unsupported_type' };
  }

  const payloadCandidate = isRecord(raw.payload)
    ? raw.payload
    : isRecord(raw.data)
      ? raw.data
      : null;
  if (!payloadCandidate) {
    return { accepted: false, cursor, reason: 'invalid_message' };
  }

  const incomingSession = sessionId(raw.session_id ?? payloadCandidate.session_id);
  // Once a backend identifies its session, an unscoped message cannot safely
  // replace it. A new socket starts with a fresh cursor and may use legacy mode.
  if (cursor.sessionId !== null && incomingSession === null) {
    return { accepted: false, cursor, reason: 'missing_session' };
  }

  const sessionChanged = incomingSession !== null && incomingSession !== cursor.sessionId;
  const incomingSequence = sequenceNumber(raw.sequence ?? payloadCandidate.sequence);
  const incomingTimestamp = timestampNumber(raw.timestamp ?? payloadCandidate.timestamp);

  if (!sessionChanged) {
    if (incomingSequence !== null && cursor.sequence !== null) {
      if (!isNewerSequence(incomingSequence, cursor.sequence)) {
        return { accepted: false, cursor, reason: 'out_of_order' };
      }
    } else if (incomingSequence === null && cursor.sequence !== null) {
      return { accepted: false, cursor, reason: 'out_of_order' };
    } else if (
      incomingSequence === null
      && incomingTimestamp !== null
      && cursor.timestamp !== null
      && incomingTimestamp <= cursor.timestamp
    ) {
      return { accepted: false, cursor, reason: 'out_of_order' };
    }
  }

  const nextCursor: TelemetryCursor = {
    sessionId: incomingSession ?? cursor.sessionId,
    sequence: incomingSequence,
    timestamp: incomingTimestamp,
  };
  return {
    accepted: true,
    cursor: nextCursor,
    payload: {
      ...payloadCandidate,
      ...(incomingSession ? { session_id: incomingSession } : {}),
      ...(incomingSequence !== null ? { sequence: incomingSequence } : {}),
      ...(incomingTimestamp !== null ? { timestamp: incomingTimestamp } : {}),
    },
    sessionChanged,
  };
}
