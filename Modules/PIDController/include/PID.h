#pragma once
#include <cstring>
#include <algorithm>

struct PID_Regulator_t {
    float ref = 0;
    float fdb = 0;
    float err[4] = {0};
    float errSum = 0;
    float kp = 0, ki = 0, kd = 0;
    float componentKp = 0, componentKi = 0, componentKd = 0;
    float componentKpMax = 0, componentKiMax = 0, componentKdMax = 0;
    float output = 0;
    float outputMax = 0;

    PID_Regulator_t() = default;
    PID_Regulator_t(float kp_, float ki_, float kd_, float pM, float iM, float dM, float oM)
        : kp(kp_), ki(ki_), kd(kd_)
        , componentKpMax(pM), componentKiMax(iM), componentKdMax(dM)
        , outputMax(oM) {}
};

class PID {
public:
    PID_Regulator_t PIDInfo{};
    void Reset();
    void Reset(PID_Regulator_t* pidRegulator);
    float PIDCalc(float target, float feedback);
    float PIDCalc(float target, float feedback, float max);
};
