/**
 * @file protocol_handler.cpp
 * @brief Task-context command validation, state transitions, and responses.
 */

#include "binary_protocol.h"
#include "main.h"
#include "robot_data.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

BinaryProtocol bp_instance;
bool bp_initialized = false;

constexpr uint8_t kChannelCount = 8U;
constexpr uint32_t kMinPwmTimeoutMs = 200U;
constexpr uint32_t kMaxPwmTimeoutMs = 2000U;
constexpr float kMinDepthCm = 0.0F;
constexpr float kMaxDepthCm = 30000.0F;
constexpr float kMaxAbsYawDeg = 360.0F;

void increment_protocol_error()
{
    taskENTER_CRITICAL();
    if (robot.protocol_errors != UINT16_MAX)
    {
        robot.protocol_errors++;
    }
    taskEXIT_CRITICAL();
}

void queue_ack(uint16_t sequence)
{
    if (!bp_send_ack(&bp_instance, sequence))
    {
        increment_protocol_error();
    }
}

void reject(uint16_t sequence, uint8_t type, uint8_t reason)
{
    increment_protocol_error();
    if (!bp_send_nack(&bp_instance, sequence, type, reason))
    {
        increment_protocol_error();
    }
}

void bp_log(const char *message)
{
    if (message == nullptr) return;
    uint16_t length = static_cast<uint16_t>(std::strlen(message));
    if (length > PROTO_MAX_PAYLOAD) length = PROTO_MAX_PAYLOAD;
    (void)bp_send_frame_priority(
        &bp_instance, ProtoMsg_LOG_MESSAGE,
        reinterpret_cast<const uint8_t *>(message), length,
        BP_TX_PRIORITY_NORMAL);
}

bool set_state_locked(RobotState state, uint32_t now)
{
    if (robot.state == state) return false;
    robot.state = state;
    robot.state_changed_ms = now;
    return true;
}

void log_state_change(bool changed, RobotState state)
{
    if (!changed) return;
    static const char *const names[] = {
        "DISARMED", "ARMED_IDLE", "ARMED_ACTIVE", "MANUAL_TEST",
        "COMM_LOST", "EMERGENCY_STOP", "FAULT",
    };
    const uint8_t index = static_cast<uint8_t>(state);
    if (index >= (sizeof(names) / sizeof(names[0]))) return;
    char message[48];
    std::snprintf(message, sizeof(message), "STATE:%s", names[index]);
    bp_log(message);
}

void neutral_locked(uint8_t reason)
{
    for (uint8_t channel = 0U; channel < kChannelCount; ++channel)
    {
        robot.pwm[channel] = ROBOT_PWM_NEUTRAL_US;
        robot.manual_pwm[channel] = ROBOT_PWM_NEUTRAL_US;
    }
    robot.manual_pwm_enabled = false;
    robot.active_test_channel = 0xFFU;
    robot.last_neutral_reason = reason;
}

template <typename T>
T decode_payload(const uint8_t *payload)
{
    T decoded{};
    std::memcpy(&decoded, payload, sizeof(decoded));
    return decoded;
}

void handle_arm(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    uint8_t failure = 0xFFU;

    taskENTER_CRITICAL();
    if (robot.estop_locked || robot.state == RobotState::EMERGENCY_STOP)
    {
        failure = ProtoNack_EstopLocked;
    }
    else if (robot.state == RobotState::DISARMED ||
             robot.state == RobotState::COMM_LOST)
    {
        robot.control_enable = false;
        robot.float_enabled = false;
        robot.angle_enabled = false;
        robot.motion_state = 0U;
        neutral_locked(ProtoNeutral_NONE);
        changed = set_state_locked(RobotState::ARMED_IDLE, now);
    }
    else
    {
        failure = ProtoNack_BadState;
    }
    taskEXIT_CRITICAL();

    if (failure != 0xFFU)
    {
        reject(sequence, ProtoMsg_ARM, failure);
        return;
    }
    log_state_change(changed, RobotState::ARMED_IDLE);
    queue_ack(sequence);
}

void handle_disarm(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    RobotState resulting_state;

    taskENTER_CRITICAL();
    robot.control_enable = false;
    robot.float_enabled = false;
    robot.angle_enabled = false;
    robot.motion_state = 0U;
    if (robot.estop_locked)
    {
        neutral_locked(ProtoNeutral_EMERGENCY_STOP);
        resulting_state = RobotState::EMERGENCY_STOP;
        changed = set_state_locked(resulting_state, now);
    }
    else
    {
        neutral_locked(ProtoNeutral_DISARM);
        resulting_state = RobotState::DISARMED;
        changed = set_state_locked(resulting_state, now);
    }
    taskEXIT_CRITICAL();

    log_state_change(changed, resulting_state);
    queue_ack(sequence);
}

void handle_emergency_stop(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed;
    taskENTER_CRITICAL();
    robot.control_enable = false;
    robot.float_enabled = false;
    robot.angle_enabled = false;
    robot.motion_state = 0U;
    robot.estop_locked = true;
    neutral_locked(ProtoNeutral_EMERGENCY_STOP);
    changed = set_state_locked(RobotState::EMERGENCY_STOP, now);
    taskEXIT_CRITICAL();

    log_state_change(changed, RobotState::EMERGENCY_STOP);
    queue_ack(sequence);
    bp_log("ESTOP");
}

void handle_reset_estop(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    taskENTER_CRITICAL();
    if (robot.estop_locked && robot.state == RobotState::EMERGENCY_STOP)
    {
        robot.estop_locked = false;
        neutral_locked(ProtoNeutral_DISARM);
        changed = set_state_locked(RobotState::DISARMED, now);
        valid = true;
    }
    taskEXIT_CRITICAL();

    if (!valid)
    {
        reject(sequence, ProtoMsg_RESET_ESTOP, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, RobotState::DISARMED);
    queue_ack(sequence);
}

void handle_enter_manual(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    taskENTER_CRITICAL();
    if (!robot.estop_locked && robot.state == RobotState::ARMED_IDLE)
    {
        robot.control_enable = false;
        robot.float_enabled = false;
        robot.angle_enabled = false;
        robot.motion_state = 0U;
        neutral_locked(ProtoNeutral_NONE);
        changed = set_state_locked(RobotState::MANUAL_TEST, now);
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_ENTER_MANUAL, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, RobotState::MANUAL_TEST);
    queue_ack(sequence);
}

void handle_exit_manual(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    taskENTER_CRITICAL();
    if (robot.state == RobotState::MANUAL_TEST)
    {
        neutral_locked(ProtoNeutral_COMMAND);
        changed = set_state_locked(RobotState::ARMED_IDLE, now);
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_EXIT_MANUAL, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, RobotState::ARMED_IDLE);
    queue_ack(sequence);
}

void handle_set_pwm(const uint8_t *payload, uint16_t sequence)
{
    const ProtoSetPwm command = decode_payload<ProtoSetPwm>(payload);
    if (command.channel >= kChannelCount)
    {
        reject(sequence, ProtoMsg_SET_PWM, ProtoNack_BadChannel);
        return;
    }
    if (command.reserved != 0U)
    {
        reject(sequence, ProtoMsg_SET_PWM, ProtoNack_InvalidValue);
        return;
    }

    int32_t min_pwm;
    int32_t max_pwm;
    taskENTER_CRITICAL();
    min_pwm = robot.pwm_test_min_us;
    max_pwm = robot.pwm_test_max_us;
    taskEXIT_CRITICAL();
    if (command.pwm_us < min_pwm || command.pwm_us > max_pwm)
    {
        reject(sequence, ProtoMsg_SET_PWM, ProtoNack_BadPwmValue);
        return;
    }

    uint32_t timeout = command.timeout_ms;
    if (timeout == 0U) timeout = ROBOT_MANUAL_PWM_TIMEOUT_MS;
    if (timeout < kMinPwmTimeoutMs || timeout > kMaxPwmTimeoutMs)
    {
        reject(sequence, ProtoMsg_SET_PWM, ProtoNack_InvalidValue);
        return;
    }

    const uint32_t now = HAL_GetTick();
    uint8_t failure = 0xFFU;
    taskENTER_CRITICAL();
    if (robot.estop_locked || robot.state != RobotState::MANUAL_TEST)
    {
        failure = robot.estop_locked ? ProtoNack_EstopLocked
                                     : ProtoNack_NotArmed;
    }
    else if (robot.active_test_channel != 0xFFU &&
             robot.active_test_channel != command.channel)
    {
        failure = ProtoNack_ChannelBusy;
    }
    else
    {
        robot.active_test_channel = command.channel;
        robot.channel_test_deadline = now + timeout;
        robot.manual_pwm[command.channel] = command.pwm_us;
        robot.manual_pwm_enabled = true;
        robot.manual_pwm_last_ms = now;
        robot.last_cmd_tick = now;
        robot.last_neutral_reason = ProtoNeutral_NONE;
    }
    taskEXIT_CRITICAL();

    if (failure != 0xFFU)
    {
        reject(sequence, ProtoMsg_SET_PWM, failure);
        return;
    }
    queue_ack(sequence);
}

void handle_set_all_neutral(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    RobotState resulting_state;
    taskENTER_CRITICAL();
    robot.control_enable = false;
    robot.float_enabled = false;
    robot.angle_enabled = false;
    robot.motion_state = 0U;
    neutral_locked(robot.estop_locked ? ProtoNeutral_EMERGENCY_STOP
                                      : ProtoNeutral_COMMAND);
    resulting_state = robot.state;
    if (!robot.estop_locked &&
        (robot.state == RobotState::MANUAL_TEST ||
         robot.state == RobotState::ARMED_ACTIVE))
    {
        resulting_state = RobotState::ARMED_IDLE;
        changed = set_state_locked(resulting_state, now);
    }
    taskEXIT_CRITICAL();
    log_state_change(changed, resulting_state);
    queue_ack(sequence);
}

void handle_float_on(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    taskENTER_CRITICAL();
    if (!robot.estop_locked &&
        (robot.state == RobotState::ARMED_IDLE ||
         robot.state == RobotState::ARMED_ACTIVE))
    {
        const float current_cm = robot.depth_m * 100.0F;
        robot.target_depth_cm = current_cm > 30.0F ? current_cm : 30.0F;
        robot.float_enabled = true;
        robot.control_enable = true;
        robot.manual_pwm_enabled = false;
        robot.active_test_channel = 0xFFU;
        robot.motion_state = 0U;
        robot.last_cmd_tick = now;
        robot.last_neutral_reason = ProtoNeutral_NONE;
        changed = set_state_locked(RobotState::ARMED_ACTIVE, now);
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_FLOAT_ON, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, RobotState::ARMED_ACTIVE);
    queue_ack(sequence);
}

void handle_float_off(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    RobotState resulting_state = RobotState::ARMED_IDLE;
    taskENTER_CRITICAL();
    if (robot.state == RobotState::ARMED_ACTIVE)
    {
        robot.float_enabled = false;
        robot.motion_state = 0U;
        if (!robot.angle_enabled)
        {
            robot.control_enable = false;
            neutral_locked(ProtoNeutral_COMMAND);
            changed = set_state_locked(RobotState::ARMED_IDLE, now);
        }
        else
        {
            resulting_state = RobotState::ARMED_ACTIVE;
        }
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_FLOAT_OFF, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, resulting_state);
    queue_ack(sequence);
}

void handle_angle_on(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    taskENTER_CRITICAL();
    if (!robot.estop_locked &&
        (robot.state == RobotState::ARMED_IDLE ||
         robot.state == RobotState::ARMED_ACTIVE))
    {
        robot.angle_enabled = true;
        robot.control_enable = true;
        robot.target_yaw = robot.yaw;
        robot.last_cmd_tick = now;
        robot.last_neutral_reason = ProtoNeutral_NONE;
        changed = set_state_locked(RobotState::ARMED_ACTIVE, now);
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_ANGLE_ON, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, RobotState::ARMED_ACTIVE);
    queue_ack(sequence);
}

void handle_angle_off(uint16_t sequence)
{
    const uint32_t now = HAL_GetTick();
    bool changed = false;
    bool valid = false;
    RobotState resulting_state = RobotState::ARMED_IDLE;
    taskENTER_CRITICAL();
    if (robot.state == RobotState::ARMED_ACTIVE)
    {
        robot.angle_enabled = false;
        if (!robot.float_enabled)
        {
            robot.control_enable = false;
            neutral_locked(ProtoNeutral_COMMAND);
            changed = set_state_locked(RobotState::ARMED_IDLE, now);
        }
        else
        {
            resulting_state = RobotState::ARMED_ACTIVE;
        }
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_ANGLE_OFF, ProtoNack_BadState);
        return;
    }
    log_state_change(changed, resulting_state);
    queue_ack(sequence);
}

void handle_set_depth(const uint8_t *payload, uint16_t sequence)
{
    const ProtoSetDepth command = decode_payload<ProtoSetDepth>(payload);
    if (!std::isfinite(command.target_depth_cm) ||
        command.target_depth_cm < kMinDepthCm ||
        command.target_depth_cm > kMaxDepthCm)
    {
        reject(sequence, ProtoMsg_SET_DEPTH, ProtoNack_InvalidValue);
        return;
    }
    bool valid = false;
    taskENTER_CRITICAL();
    if (robot.state == RobotState::ARMED_ACTIVE && robot.float_enabled &&
        !robot.estop_locked)
    {
        robot.target_depth_cm = command.target_depth_cm;
        robot.last_cmd_tick = HAL_GetTick();
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_SET_DEPTH, ProtoNack_BadState);
        return;
    }
    queue_ack(sequence);
}

void handle_set_yaw(const uint8_t *payload, uint16_t sequence)
{
    const ProtoSetYaw command = decode_payload<ProtoSetYaw>(payload);
    if (!std::isfinite(command.target_yaw_deg) ||
        std::fabs(command.target_yaw_deg) > kMaxAbsYawDeg)
    {
        reject(sequence, ProtoMsg_SET_YAW, ProtoNack_InvalidValue);
        return;
    }
    constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
    bool valid = false;
    taskENTER_CRITICAL();
    if (robot.state == RobotState::ARMED_ACTIVE && !robot.estop_locked)
    {
        robot.target_yaw = command.target_yaw_deg * kDegToRad;
        robot.angle_enabled = true;
        robot.control_enable = true;
        robot.last_cmd_tick = HAL_GetTick();
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_SET_YAW, ProtoNack_BadState);
        return;
    }
    queue_ack(sequence);
}

void handle_set_motion(const uint8_t *payload, uint16_t sequence)
{
    const ProtoSetMotion command = decode_payload<ProtoSetMotion>(payload);
    if (command.motion_state > 7U)
    {
        reject(sequence, ProtoMsg_SET_MOTION, ProtoNack_InvalidValue);
        return;
    }
    bool valid = false;
    taskENTER_CRITICAL();
    if (robot.state == RobotState::ARMED_ACTIVE && robot.float_enabled &&
        !robot.estop_locked)
    {
        robot.motion_state = command.motion_state;
        robot.last_cmd_tick = HAL_GetTick();
        valid = true;
    }
    taskEXIT_CRITICAL();
    if (!valid)
    {
        reject(sequence, ProtoMsg_SET_MOTION, ProtoNack_BadState);
        return;
    }
    queue_ack(sequence);
}

void handle_heartbeat(const uint8_t *payload, uint16_t sequence)
{
    const ProtoHeartbeat heartbeat = decode_payload<ProtoHeartbeat>(payload);
    if (heartbeat.heartbeat_timeout_ms < 200U ||
        heartbeat.heartbeat_timeout_ms > 5000U)
    {
        reject(sequence, ProtoMsg_HEARTBEAT, ProtoNack_InvalidValue);
        return;
    }

    const uint32_t now = HAL_GetTick();
    bool changed = false;
    taskENTER_CRITICAL();
    robot.heartbeat_timeout_ms = heartbeat.heartbeat_timeout_ms;
    robot.last_heartbeat_ms = now;
    if (robot.state == RobotState::COMM_LOST)
    {
        changed = set_state_locked(RobotState::DISARMED, now);
    }
    taskEXIT_CRITICAL();
    log_state_change(changed, RobotState::DISARMED);
    if (changed) bp_log("LINK RESTORED");

    ProtoHeartbeatAck ack{};
    taskENTER_CRITICAL();
    ack.safety_state = static_cast<uint8_t>(robot.state);
    ack.uptime_s = static_cast<uint16_t>(robot.uptime_s & 0xFFFFU);
    ack.error_count = robot.protocol_errors;
    taskEXIT_CRITICAL();
    if (!bp_send_frame_priority(
            &bp_instance, ProtoMsg_HEARTBEAT_ACK,
            reinterpret_cast<const uint8_t *>(&ack), sizeof(ack),
            BP_TX_PRIORITY_HIGH))
    {
        increment_protocol_error();
    }
}

} // namespace

extern "C" BinaryProtocol *bp_get_instance(void)
{
    if (!bp_initialized)
    {
        bp_init(&bp_instance);
        bp_initialized = true;
    }
    return &bp_instance;
}

extern "C" bool bp_queue_status_report(BinaryProtocol *bp,
                                        BpTxPriority priority)
{
    if (bp == nullptr) return false;
    ProtoStatusReport report{};
    uint32_t errors;
    taskENTER_CRITICAL();
    report.safety_state = static_cast<uint8_t>(robot.state);
    if (robot.control_enable) report.flags |= 0x01U;
    if (robot.float_enabled) report.flags |= 0x02U;
    if (robot.angle_enabled) report.flags |= 0x04U;
    if (robot.manual_pwm_enabled) report.flags |= 0x08U;
    if (robot.estop_locked) report.flags |= 0x10U;
    for (uint8_t channel = 0U; channel < kChannelCount; ++channel)
    {
        report.pwm[channel] = static_cast<int16_t>(robot.pwm[channel]);
    }
    errors = robot.protocol_errors;
    report.heartbeat_missed = robot.heartbeat_missed;
    report.neutral_reason = robot.last_neutral_reason;
    report.active_channel = robot.active_test_channel;
    taskEXIT_CRITICAL();

    errors += bp->crc_errors + bp->sync_losses + bp->tx_dropped_frames +
              bp->tx_send_failures;
    report.error_count = static_cast<uint16_t>(
        errors > UINT16_MAX ? UINT16_MAX : errors);
    return bp_send_frame_priority(
        bp, ProtoMsg_STATUS_REPORT,
        reinterpret_cast<const uint8_t *>(&report), sizeof(report), priority);
}

extern "C" bool bp_queue_sensor_report(BinaryProtocol *bp,
                                        BpTxPriority priority)
{
    if (bp == nullptr) return false;
    ProtoSensorReport report{};
    taskENTER_CRITICAL();
    report.depth_m = robot.depth_m;
    report.pressure_mbar = robot.pressure_mbar;
    report.water_temp_c = robot.water_temp_c;
    report.yaw = robot.yaw;
    report.pitch = robot.pitch;
    report.roll = robot.roll;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        report.accel[i] = robot.accel[i];
        report.gyro[i] = robot.gyro[i];
        report.mag[i] = robot.mag[i];
    }
    report.yaw_v = robot.yaw_v;
    report.pitch_v = robot.pitch_v;
    report.roll_v = robot.roll_v;
    taskEXIT_CRITICAL();
    return bp_send_frame_priority(
        bp, ProtoMsg_SENSOR_REPORT,
        reinterpret_cast<const uint8_t *>(&report), sizeof(report), priority);
}

extern "C" void bp_dispatch_frame(BinaryProtocol *bp, uint8_t type,
                                  uint16_t sequence, const uint8_t *payload,
                                  uint16_t payload_len)
{
    (void)bp;
    uint16_t expected_length = 0U;
    if (!bp_expected_command_payload_length(type, &expected_length))
    {
        reject(sequence, type, ProtoNack_UnsupportedMessage);
        return;
    }
    if (payload_len != expected_length ||
        (expected_length > 0U && payload == nullptr))
    {
        reject(sequence, type, ProtoNack_InvalidPayloadLength);
        return;
    }

    switch (type)
    {
    case ProtoMsg_NOP:
        queue_ack(sequence);
        break;
    case ProtoMsg_HEARTBEAT:
        handle_heartbeat(payload, sequence);
        break;
    case ProtoMsg_ARM:
        handle_arm(sequence);
        break;
    case ProtoMsg_DISARM:
        handle_disarm(sequence);
        break;
    case ProtoMsg_EMERGENCY_STOP:
        handle_emergency_stop(sequence);
        break;
    case ProtoMsg_RESET_ESTOP:
        handle_reset_estop(sequence);
        break;
    case ProtoMsg_ENTER_MANUAL:
        handle_enter_manual(sequence);
        break;
    case ProtoMsg_EXIT_MANUAL:
        handle_exit_manual(sequence);
        break;
    case ProtoMsg_SET_PWM:
        handle_set_pwm(payload, sequence);
        break;
    case ProtoMsg_SET_ALL_NEUTRAL:
        handle_set_all_neutral(sequence);
        break;
    case ProtoMsg_FLOAT_ON:
        handle_float_on(sequence);
        break;
    case ProtoMsg_FLOAT_OFF:
        handle_float_off(sequence);
        break;
    case ProtoMsg_ANGLE_ON:
        handle_angle_on(sequence);
        break;
    case ProtoMsg_ANGLE_OFF:
        handle_angle_off(sequence);
        break;
    case ProtoMsg_SET_DEPTH:
        handle_set_depth(payload, sequence);
        break;
    case ProtoMsg_SET_YAW:
        handle_set_yaw(payload, sequence);
        break;
    case ProtoMsg_SET_MOTION:
        handle_set_motion(payload, sequence);
        break;
    case ProtoMsg_REQUEST_STATUS:
        if (!bp_queue_status_report(&bp_instance, BP_TX_PRIORITY_HIGH))
            increment_protocol_error();
        break;
    case ProtoMsg_REQUEST_SENSORS:
        if (!bp_queue_sensor_report(&bp_instance, BP_TX_PRIORITY_HIGH))
            increment_protocol_error();
        break;
    default:
        reject(sequence, type, ProtoNack_UnsupportedMessage);
        break;
    }
}
