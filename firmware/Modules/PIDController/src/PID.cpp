#include "PID.h"
#include <algorithm>
#include <cmath>

void PID::Reset() {
    PIDInfo.ref = 0;
    PIDInfo.fdb = 0;
    PIDInfo.err[0] = 0;
    PIDInfo.err[1] = 0;
    PIDInfo.err[2] = 0;
    PIDInfo.err[3] = 0;
    PIDInfo.errSum = 0;

    PIDInfo.componentKp = 0;
    PIDInfo.componentKi = 0;
    PIDInfo.componentKd = 0;

    PIDInfo.output = 0;
}

void PID::Reset(PID_Regulator_t* pidRegulator) {
    if (pidRegulator != nullptr) PIDInfo = *pidRegulator;
}

float PID::PIDCalc(float target, float feedback) {
    if (!std::isfinite(target) ||
        !std::isfinite(feedback) ||
        !std::isfinite(PIDInfo.kp) ||
        !std::isfinite(PIDInfo.ki) ||
        !std::isfinite(PIDInfo.kd) ||
        !std::isfinite(PIDInfo.componentKpMax) ||
        !std::isfinite(PIDInfo.componentKiMax) ||
        !std::isfinite(PIDInfo.componentKdMax) ||
        !std::isfinite(PIDInfo.outputMax) ||
        PIDInfo.componentKpMax < 0.0F ||
        PIDInfo.componentKiMax < 0.0F ||
        PIDInfo.componentKdMax < 0.0F ||
        PIDInfo.outputMax < 0.0F) {
        Reset();
        return 0.0F;
    }

    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[3] * PIDInfo.kp;
    if (PIDInfo.ki != 0.0F && PIDInfo.componentKiMax > 0.0F) {
        PIDInfo.errSum += PIDInfo.err[3];
        const float integral_error_limit =
            PIDInfo.componentKiMax / std::fabs(PIDInfo.ki);
        PIDInfo.errSum = std::clamp(
            PIDInfo.errSum,
            -integral_error_limit,
            integral_error_limit);
    } else {
        PIDInfo.errSum = 0.0F;
    }

    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;

    if (!std::isfinite(PIDInfo.componentKp) ||
        !std::isfinite(PIDInfo.componentKi) ||
        !std::isfinite(PIDInfo.componentKd)) {
        Reset();
        return 0.0F;
    }

    PIDInfo.componentKp = std::clamp(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    PIDInfo.componentKi = std::clamp(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    PIDInfo.componentKd = std::clamp(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);

    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    PIDInfo.output = std::clamp(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);

    PIDInfo.err[2] = PIDInfo.err[3];
    return PIDInfo.output;
}

float PID::PIDCalc(float target, float feedback, float max) {
    PIDInfo.outputMax = max;
    return PID::PIDCalc(target, feedback);
}
