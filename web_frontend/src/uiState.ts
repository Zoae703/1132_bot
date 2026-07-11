export type ReadinessTone = 'ok' | 'warn' | 'danger';

export interface ReadinessStatus {
  serial_connected: boolean;
  stm32_online: boolean;
  status_stale: boolean;
  backend_motion_inhibited?: boolean;
  backend_motion_inhibit_reason?: string | null;
}

export interface FreshnessStatus {
  stm32_online: boolean;
  sensors_stale: boolean;
}

export interface PwmPermissionStatus extends ReadinessStatus {
  safety_state: number;
  estop_locked: boolean;
}

export function linkReady(
  connected: boolean,
  status: ReadinessStatus | null,
): boolean {
  return Boolean(
    connected
    && status?.serial_connected
    && status.stm32_online
    && !status.status_stale,
  );
}

export function backendReady(
  connected: boolean,
  status: ReadinessStatus | null,
): boolean {
  return linkReady(connected, status) && !status?.backend_motion_inhibited;
}

export function readinessView(
  connected: boolean,
  status: ReadinessStatus | null,
  capabilitiesReady: boolean,
): { tone: ReadinessTone; text: string } {
  if (!connected) {
    return { tone: 'danger', text: 'WebSocket 已断开，页面离线，所有控制已锁定' };
  }
  if (!status) {
    return { tone: 'warn', text: '等待状态数据，控制按钮暂不可用' };
  }
  if (!status.serial_connected) {
    return { tone: 'danger', text: '串口链路离线，控制已锁定' };
  }
  if (!status.stm32_online) {
    return { tone: 'danger', text: 'STM32 离线，控制已锁定' };
  }
  if (status.status_stale) {
    return { tone: 'warn', text: 'STM32 在线，但状态数据已过期，控制已锁定' };
  }
  if (status.backend_motion_inhibited) {
    const reason = status.backend_motion_inhibit_reason ?? '安全命令结果不确定';
    return { tone: 'danger', text: `后端运动锁定：${reason}；请先确认上锁回中` };
  }
  if (!capabilitiesReady) {
    return { tone: 'warn', text: '安全能力配置不可用，控制已锁定' };
  }
  return { tone: 'ok', text: '链路与状态数据正常，可以按当前安全状态执行操作' };
}

export function sensorsFreshnessMessage(
  connected: boolean,
  status: FreshnessStatus,
): string | null {
  if (!connected) return 'WebSocket 已断开，以下仅为断线前缓存，不代表当前传感器状态。';
  if (!status.stm32_online) return 'STM32 已离线，以下仅为最后一次缓存。';
  if (status.sensors_stale) return '传感器数据已过期，以下数值不是实时数据。';
  return null;
}

export function pwmControlAvailability({
  connected,
  status,
  capabilitiesReady,
  safetyLocked,
  requestPending,
}: {
  connected: boolean;
  status: PwmPermissionStatus | null;
  capabilitiesReady: boolean;
  safetyLocked: boolean;
  requestPending: boolean;
}): { canTest: boolean; canNeutral: boolean } {
  const link = linkReady(connected, status);
  const motionReady = backendReady(connected, status);
  return {
    canTest: Boolean(
      motionReady
      && capabilitiesReady
      && status?.safety_state === 3
      && status.estop_locked === false
      && !safetyLocked
      && !requestPending,
    ),
    // Neutral is a safety action and remains available while ESTOP is latched.
    canNeutral: Boolean(link && !requestPending),
  };
}

export function shouldClearLocalRequest(connected: boolean, safetyLocked: boolean): boolean {
  return !connected || safetyLocked;
}

export function pwmDisplayPair(
  confirmed: number[],
  requested: number[],
  channel: number,
): { confirmed: number | undefined; requested: number | undefined } {
  return { confirmed: confirmed[channel], requested: requested[channel] };
}

export function apiErrorMessage(detail: unknown, fallback: string): string {
  if (typeof detail === 'string' && detail.trim()) return detail;
  if (Array.isArray(detail)) {
    const messages = detail
      .map(item => {
        if (typeof item === 'string') return item;
        if (item && typeof item === 'object' && 'msg' in item && typeof item.msg === 'string') return item.msg;
        return null;
      })
      .filter((item): item is string => Boolean(item));
    if (messages.length) return messages.join('；');
  }
  if (detail && typeof detail === 'object') {
    try {
      return JSON.stringify(detail);
    } catch {
      return fallback;
    }
  }
  return fallback;
}
