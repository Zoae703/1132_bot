#pragma once

class MahonyAHRS {
public:
    MahonyAHRS(float sample_freq = 200.0f, float kp = 5.0f, float ki = 0.0f);

    // Main update: 9-DOF (gyro+accel+mag)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);

    // Fallback: 6-DOF (gyro+accel only, no mag)
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az);

    // Quaternion → Euler angles (radians). Static — no instance needed.
    static void get_angle(const float q[4], float* yaw, float* pitch, float* roll);

    // Accessors
    float quat_w() const { return q_[0]; }
    float quat_x() const { return q_[1]; }
    float quat_y() const { return q_[2]; }
    float quat_z() const { return q_[3]; }
    void  get_quat(float out[4]) const;
    float yaw()   const;
    float pitch() const;
    float roll()  const;

    void reset();

private:
    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float twoKp_;
    float twoKi_;
    float integralFBx_ = 0, integralFBy_ = 0, integralFBz_ = 0;
    float sampleFreq_;

    static float invSqrt(float x);
};
