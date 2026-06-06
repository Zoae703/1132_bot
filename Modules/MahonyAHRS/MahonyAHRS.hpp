#pragma once
#include "include/MahonyAHRS.h"
#include "DataBus.hpp"

class MahonyAHRSModule {
public:
    MahonyAHRSModule(float sample_freq = 200.0f, float kp = 5.0f, float ki = 0.0f)
        : ahrs_(sample_freq, kp, ki) {}

    void Init() { ahrs_.reset(); }

    void Update() {
        // Read raw data from DataBus
        float gx = robot.gyro[0], gy = robot.gyro[1], gz = robot.gyro[2];
        float ax = robot.accel[0], ay = robot.accel[1], az = robot.accel[2];
        float mx = robot.mag[0], my = robot.mag[1], mz = robot.mag[2];

        // Run fusion
        ahrs_.update(gx, gy, gz, ax, ay, az, mx, my, mz);

        // Extract quaternion
        ahrs_.get_quat(robot.quat);

        // Extract Euler angles
        MahonyAHRS::get_angle(robot.quat, &robot.yaw, &robot.pitch, &robot.roll);

        // Angular velocities (from gyro, already in rad/s)
        robot.yaw_v   = gz;
        robot.pitch_v = gy;
        robot.roll_v  = gx;
    }

private:
    MahonyAHRS ahrs_;
};
