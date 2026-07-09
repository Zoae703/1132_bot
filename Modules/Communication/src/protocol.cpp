#include "protocol.h"
#include "robot_data.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include <cstdlib>
#include <cerrno>

extern "C" UART_HandleTypeDef huart6;

#define INIT_PWM 1610
#define COMM_PI  3.14159265358979323846f
#define COMM_QUEUE_DEPTH 4U

// Motion states (match RobotData::motion_state encoding)
enum { ST_STOP = 0, ST_FLOAT, ST_FRONT, ST_BACK, ST_LEFT, ST_RIGHT, ST_CLOCKWISE, ST_ANTICLOCKWISE };

struct CommFrame {
    uint16_t size;
    uint8_t data[COMM_BUF_LEN];
};

uint8_t CommRxBuffer[COMM_BUF_LEN];
static osMessageQueueId_t comm_queue = nullptr;
static const osMessageQueueAttr_t comm_queue_attributes = {
    .name = "commRxQueue",
};

static float deg2rad(float deg) { return deg * COMM_PI / 180.0f; }

static int32_t clamp_pwm(int32_t pwm) {
    if (pwm < 1000) return 1000;
    if (pwm > 2000) return 2000;
    return pwm;
}

static bool parse_pwm_token(const char* tok, int32_t* pwm) {
    if (tok == nullptr || pwm == nullptr || *tok == '\0') {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    long value = strtol(tok, &end, 10);
    if (errno != 0 || end == tok) {
        return false;
    }
    while (*end == ' ' || *end == '\r' || *end == '\n' || *end == '\t') {
        ++end;
    }
    if (*end != '\0') {
        return false;
    }

    *pwm = clamp_pwm((int32_t)value);
    return true;
}

static void set_neutral_pwm_locked() {
    for (int i = 0; i < 8; ++i) {
        robot.pwm[i] = INIT_PWM;
    }
}

static uint8_t motion_of(char c) {
    switch (c) {
        case 'W': return ST_FRONT;
        case 'S': return ST_BACK;
        case 'A': return ST_LEFT;
        case 'D': return ST_RIGHT;
        case 'Z': return ST_FLOAT;
        case 'E': return ST_CLOCKWISE;
        case 'Q': return ST_ANTICLOCKWISE;
        default:  return 0xFF;
    }
}

static void handle_tes(char* buf) {
    if (robot.float_enabled || robot.angle_enabled) {
        return;
    }

    int32_t next_pwm[8];
    for (int i = 0; i < 8; ++i) {
        next_pwm[i] = INIT_PWM;
    }

    char* args = buf + 3;
    if (*args == ':' || *args == ',' || *args == ' ') {
        ++args;
    }

    char* tok = strtok(args, ",");
    int i = 0;
    while (tok && i < 8) {
        if (!parse_pwm_token(tok, &next_pwm[i])) {
            return;
        }
        tok = strtok(nullptr, ",");
        i++;
    }

    uint32_t now = HAL_GetTick();
    taskENTER_CRITICAL();
    for (int ch = 0; ch < 8; ++ch) {
        robot.pwm[ch] = next_pwm[ch];
    }
    robot.manual_pwm_enabled = true;
    robot.manual_pwm_last_ms = now;
    taskEXIT_CRITICAL();
}

static void handle_off() {
    taskENTER_CRITICAL();
    robot.float_enabled = false;
    robot.angle_enabled = false;
    robot.manual_pwm_enabled = false;
    set_neutral_pwm_locked();
    robot.target_depth_cm = 30;
    robot.motion_state = ST_STOP;
    taskEXIT_CRITICAL();
}

static void process_frame(char* buf) {
    // --- command table (matched regardless of float flag) ---
    if (strncmp(buf, "ACL", 3) == 0) {              // angle close-loop ON/OF
        if (robot.float_enabled) {
            const char* d = buf + 4;
            if (strncmp(d, "ON", 2) == 0) { robot.angle_enabled = true; robot.target_yaw = robot.yaw; }
            else if (strncmp(d, "OF", 2) == 0) { robot.angle_enabled = false; }
        }
    }
    if (strncmp(buf, "ANG", 3) == 0) {              // set yaw target (deg)
        if (robot.angle_enabled && robot.float_enabled)
            robot.target_yaw = deg2rad((float)atoi(buf + 4));
    }
    if (strncmp(buf, "TES", 3) == 0) {              // direct PWM test
        handle_tes(buf);
        return;
    }
    if (strncmp(buf, "OFF", 3) == 0) {
        handle_off();
        return;
    }

    if (robot.float_enabled) {
        if (strncmp(buf, "DN", 2) == 0)       { robot.target_depth_cm += 1; return; }
        else if (strncmp(buf, "UP", 2) == 0)  { robot.target_depth_cm -= 1; return; }
        else if (motion_of(buf[0]) != 0xFF)   { robot.motion_state = motion_of(buf[0]); }
        else if (strncmp(buf, "RPY:", 4) == 0) {
            if (strncmp(buf, "RPY:ON", 6) == 0)       robot.angle_enabled = true;
            else if (strncmp(buf, "RPY:OFF", 7) == 0) robot.angle_enabled = false;
        }
        else if (strncmp(buf, "H:", 2) == 0) {
            robot.target_depth_cm = (float)atoi(buf + 2);
        }
    } else {
        if (strncmp(buf, "ON", 2) == 0) {
            float depth_m;
            taskENTER_CRITICAL();
            depth_m = robot.depth_m;
            taskEXIT_CRITICAL();

            float cur = depth_m * 100.0f;             // m -> cm
            float target_depth_cm = (cur - 1.0f > 30.0f) ? (cur - 1.0f) : 30.0f;

            taskENTER_CRITICAL();
            robot.float_enabled = true;
            robot.manual_pwm_enabled = false;
            set_neutral_pwm_locked();
            robot.target_depth_cm = target_depth_cm;
            taskEXIT_CRITICAL();
        }
    }
}

void Comm_Init() {
    if (comm_queue == nullptr) {
        comm_queue = osMessageQueueNew(COMM_QUEUE_DEPTH, sizeof(CommFrame), &comm_queue_attributes);
        if (comm_queue == nullptr) {
            Error_Handler();
            return;
        }
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, CommRxBuffer, COMM_BUF_LEN);
}

void Comm_OnRxEvent(uint16_t size) {
    if (size >= COMM_BUF_LEN) size = COMM_BUF_LEN - 1;
    if (comm_queue != nullptr) {
        CommFrame frame;
        frame.size = size;
        memcpy(frame.data, CommRxBuffer, size);
        frame.data[size] = '\0';
        (void)osMessageQueuePut(comm_queue, &frame, 0U, 0U);
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, CommRxBuffer, COMM_BUF_LEN);
}

void Comm_Process() {
    if (comm_queue == nullptr) return;

    CommFrame frame;
    while (osMessageQueueGet(comm_queue, &frame, nullptr, 0U) == osOK) {
        if (frame.size >= COMM_BUF_LEN) {
            frame.size = COMM_BUF_LEN - 1;
        }
        frame.data[frame.size] = '\0';
        process_frame((char*)frame.data);
    }
}
