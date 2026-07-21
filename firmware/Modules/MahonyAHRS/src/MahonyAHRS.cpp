#include "MahonyAHRS.h"
#include <math.h>

MahonyAHRS::MahonyAHRS(float sample_freq, float kp, float ki)
    : twoKp_(2.0f * kp), twoKi_(2.0f * ki), sampleFreq_(sample_freq) {}

void MahonyAHRS::reset() {
    q_[0] = 1.0f; q_[1] = 0.0f; q_[2] = 0.0f; q_[3] = 0.0f;
    integralFBx_ = integralFBy_ = integralFBz_ = 0.0f;
}

void MahonyAHRS::get_quat(float out[4]) const {
    out[0] = q_[0]; out[1] = q_[1]; out[2] = q_[2]; out[3] = q_[3];
}

void MahonyAHRS::get_angle(const float q[4], float* yaw, float* pitch, float* roll) {
    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                     2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                     2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

float MahonyAHRS::yaw() const   { float y,p,r; get_angle(q_, &y, &p, &r); return y; }
float MahonyAHRS::pitch() const { float y,p,r; get_angle(q_, &y, &p, &r); return p; }
float MahonyAHRS::roll() const  { float y,p,r; get_angle(q_, &y, &p, &r); return r; }

void MahonyAHRS::update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        q0q0 = q_[0] * q_[0];
        q0q1 = q_[0] * q_[1];
        q0q2 = q_[0] * q_[2];
        q0q3 = q_[0] * q_[3];
        q1q1 = q_[1] * q_[1];
        q1q2 = q_[1] * q_[2];
        q1q3 = q_[1] * q_[3];
        q2q2 = q_[2] * q_[2];
        q2q3 = q_[2] * q_[3];
        q3q3 = q_[3] * q_[3];

        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        if (twoKi_ > 0.0f) {
            integralFBx_ += twoKi_ * halfex * (1.0f / sampleFreq_);
            integralFBy_ += twoKi_ * halfey * (1.0f / sampleFreq_);
            integralFBz_ += twoKi_ * halfez * (1.0f / sampleFreq_);
            gx += integralFBx_;
            gy += integralFBy_;
            gz += integralFBz_;
        } else {
            integralFBx_ = 0.0f;
            integralFBy_ = 0.0f;
            integralFBz_ = 0.0f;
        }

        gx += twoKp_ * halfex;
        gy += twoKp_ * halfey;
        gz += twoKp_ * halfez;
    }

    gx *= (0.5f * (1.0f / sampleFreq_));
    gy *= (0.5f * (1.0f / sampleFreq_));
    gz *= (0.5f * (1.0f / sampleFreq_));
    qa = q_[0]; qb = q_[1]; qc = q_[2];
    q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
    q_[1] += (qa * gx + qc * gz - q_[3] * gy);
    q_[2] += (qa * gy - qb * gz + q_[3] * gx);
    q_[3] += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] *= recipNorm; q_[1] *= recipNorm; q_[2] *= recipNorm; q_[3] *= recipNorm;
}

void MahonyAHRS::updateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q_[1] * q_[3] - q_[0] * q_[2];
        halfvy = q_[0] * q_[1] + q_[2] * q_[3];
        halfvz = q_[0] * q_[0] - 0.5f + q_[3] * q_[3];

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (twoKi_ > 0.0f) {
            integralFBx_ += twoKi_ * halfex * (1.0f / sampleFreq_);
            integralFBy_ += twoKi_ * halfey * (1.0f / sampleFreq_);
            integralFBz_ += twoKi_ * halfez * (1.0f / sampleFreq_);
            gx += integralFBx_;
            gy += integralFBy_;
            gz += integralFBz_;
        } else {
            integralFBx_ = 0.0f;
            integralFBy_ = 0.0f;
            integralFBz_ = 0.0f;
        }

        gx += twoKp_ * halfex;
        gy += twoKp_ * halfey;
        gz += twoKp_ * halfez;
    }

    gx *= (0.5f * (1.0f / sampleFreq_));
    gy *= (0.5f * (1.0f / sampleFreq_));
    gz *= (0.5f * (1.0f / sampleFreq_));
    qa = q_[0]; qb = q_[1]; qc = q_[2];
    q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
    q_[1] += (qa * gx + qc * gz - q_[3] * gy);
    q_[2] += (qa * gy - qb * gz + q_[3] * gx);
    q_[3] += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] *= recipNorm; q_[1] *= recipNorm; q_[2] *= recipNorm; q_[3] *= recipNorm;
}

// Fast inverse square-root (Quake)
float MahonyAHRS::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
