#include "protocol.h"
#include "robot_data.hpp"
#include <cstring>
#include <cstdlib>

extern "C" UART_HandleTypeDef huart6;

#define INIT_PWM 1610
#define COMM_PI  3.14159265358979323846f

// Motion states (match RobotData::motion_state encoding)
enum { ST_STOP = 0, ST_FLOAT, ST_FRONT, ST_BACK, ST_LEFT, ST_RIGHT, ST_CLOCKWISE, ST_ANTICLOCKWISE };

uint8_t CommRxBuffer[COMM_BUF_LEN];
static uint8_t comm_proc[COMM_BUF_LEN];
static volatile bool comm_ready = false;

static float deg2rad(float deg) { return deg * COMM_PI / 180.0f; }

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

void Comm_Init() {
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, CommRxBuffer, COMM_BUF_LEN);
}

void Comm_OnRxEvent(uint16_t size) {
    if (size >= COMM_BUF_LEN) size = COMM_BUF_LEN - 1;
    memcpy(comm_proc, CommRxBuffer, size);
    comm_proc[size] = '\0';
    comm_ready = true;
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, CommRxBuffer, COMM_BUF_LEN);
}

void Comm_Process() {
    if (!comm_ready) return;
    comm_ready = false;

    char* buf = (char*)comm_proc;

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
        if (!robot.float_enabled && !robot.angle_enabled) {
            char* tok = strtok(buf + 4, ",");
            int i = 0;
            while (tok && i < 8) { robot.pwm[i] = atoi(tok); tok = strtok(nullptr, ","); i++; }
        }
    }

    if (robot.float_enabled) {
        if (strncmp(buf, "DN", 2) == 0)       { robot.target_depth_cm += 1; return; }
        else if (strncmp(buf, "UP", 2) == 0)  { robot.target_depth_cm -= 1; return; }
        else if (motion_of(buf[0]) != 0xFF)   { robot.motion_state = motion_of(buf[0]); }
        else if (strncmp(buf, "OFF", 3) == 0) {
            robot.float_enabled = false;
            robot.angle_enabled = false;
            for (int i = 0; i < 8; ++i) robot.pwm[i] = INIT_PWM;
            robot.target_depth_cm = 30;
            robot.motion_state = ST_STOP;
        }
        else if (strncmp(buf, "RPY:", 4) == 0) {
            if (strncmp(buf, "RPY:ON", 6) == 0)       robot.angle_enabled = true;
            else if (strncmp(buf, "RPY:OFF", 7) == 0) robot.angle_enabled = false;
        }
        else if (strncmp(buf, "H:", 2) == 0) {
            robot.target_depth_cm = (float)atoi(buf + 2);
        }
    } else {
        if (strncmp(buf, "ON", 2) == 0) {
            robot.float_enabled = true;
            for (int i = 0; i < 8; ++i) robot.pwm[i] = INIT_PWM;
            float cur = robot.depth_m * 100.0f;             // m -> cm
            robot.target_depth_cm = (cur - 1.0f > 30.0f) ? (cur - 1.0f) : 30.0f;
        }
    }
}
