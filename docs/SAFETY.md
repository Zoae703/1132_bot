# Safety Architecture

## State Machine

| State | PWM Output | ARM Possible | Enter Manual | Notes |
|-------|-----------|-------------|-------------|-------|
| DISARMED | All 1500us | Yes | No (ARM first) | Power-on default |
| ARMED_IDLE | All 1500us | N/A | Yes | Armed but no output |
| ARMED_ACTIVE | PID cascade | N/A | No | Float/angle control running |
| MANUAL_TEST | Single channel only | N/A | N/A | One channel at a time |
| COMM_LOST | All 1500us | Yes (re-arms to DISARMED) | No | Auto-recover on next heartbeat |
| EMERGENCY_STOP | All 1500us | **No** (locked) | **No** | Requires RESET_ESTOP |
| FAULT | All 1500us | No | No | Requires power cycle |

## Layers of Safety

### Layer 1: STM32 State Machine (always active)
- State transitions validated on every command
- PWM clamped to state-appropriate range
- Per-command timeout enforced
- Heartbeat timeout enforced
- ESTOP latched in hardware

### Layer 2: Web Backend Validation (active when web console used)
- Input validation (channel, PWM range, duration)
- State validation before forwarding commands
- Rate limiting (300ms between PWM commands)
- Last control client disconnect forces neutral, exits manual mode, and disarms
- Requested PWM is never displayed as confirmed until a status report arrives
- Ordinary commands are single-flight; ESTOP has an independent priority path
- Startup and shutdown both request confirmed neutral and DISARM
- MOTOR_TEST, WEB_MOTION, and GAMEPAD use one exclusive ControlArbiter
- One gamepad WebSocket lease; duplicate or out-of-order frames are discarded
- Gamepad 300ms timeout sends zero and requires center before resuming
- Gamepad 1000ms timeout or disconnect exits the mode and DISARMs

### Layer 3: Physical Safety
- Power-on default: all PWM = 1500us
- PCA9685 initialized to neutral before any other operation
- I2C bus mutex prevents concurrent access
- IWDG watchdog (1s timeout, refreshed in sensorTask)

## Emergency Procedures

### Emergency Stop (Web Console)
1. Click the red **EMERGENCY STOP** button (top-right, always visible)
2. Confirm the action
3. All PWM channels immediately return to 1500us
4. System enters EMERGENCY_STOP state (latched)
5. All PWM control buttons are disabled

### Emergency Stop (STM32 Hardware)
- Send binary ESTOP message: `encode_frame(MsgType.EMERGENCY_STOP, seq)`
- Power cycle the STM32 (last resort)

### Recovery from ESTOP
1. Verify all motors are safe
2. Click **RESET ESTOP** or send `RESET_ESTOP` message
3. System transitions to DISARMED
4. ARM again when ready

`DISARM` never clears a latched ESTOP. Only `RESET_ESTOP` may leave
`EMERGENCY_STOP`; this is enforced by the firmware and mirrored by the simulator.

## Communication Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| Orange Pi → STM32 heartbeat lost | 1000ms timeout | STM32 → COMM_LOST, all PWM neutral |
| Orange Pi process killed | Heartbeat stops | Same as above |
| Serial cable disconnected | UART idle interrupt stops | Same as above |
| Orange Pi power loss | Heartbeat stops | Same as above |
| STM32 I2C bus failure | PCA9685 write error | → ESTOP |
| Last WebSocket client disconnect | Backend connection tracking | Immediate neutral, exit manual mode, DISARM; serial heartbeat continues for monitoring |
| Gamepad frame age reaches 300ms | Gamepad command pump | Six-axis zero; nonzero recovery locked until controls center |
| Gamepad frame age reaches 1000ms | Gamepad command pump | Exit GAMEPAD, zero, disable body control, DISARM |
| Gamepad unplug/client close/network socket close | Lease WebSocket | Immediate safe GAMEPAD shutdown and DISARM |

## Pre-Flight Checklist

Before any in-water testing:

- [ ] Oscilloscope on all 8 PCA9685 channels: verify 1500us neutral
- [ ] ARM → verify PWM stays 1500us
- [ ] ESTOP → verify all PWM immediately 1500us
- [ ] ESTOP → verify ARM is rejected
- [ ] RESET_ESTOP → verify transition to DISARMED
- [ ] Enter manual test → single channel PWM test 1520us/500ms
- [ ] Verify other 7 channels stay at 1500us
- [ ] Verify PWM returns to 1500us after timeout
- [ ] Disconnect serial → verify COMM_LOST within 1s
- [ ] Reconnect → verify system recovers
- [ ] Confirm USART6 contains framed binary data only (no interleaved log text)
- [ ] IWDG watchdog active (check with debugger)
