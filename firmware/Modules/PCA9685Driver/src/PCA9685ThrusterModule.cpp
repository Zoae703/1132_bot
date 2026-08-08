#include "PCA9685ThrusterModule.hpp"

PCA9685ThrusterModule::PCA9685ThrusterModule(I2C_HandleTypeDef *hi2c)
    : driver_(hi2c)
    , manager_(driver_)
{}

void PCA9685ThrusterModule::Init()
{
    /* Reset state so Start() begins from UNINITIALIZED.
     * No hardware I/O here — that happens in Start() → ServiceInitializing(). */
    manager_.Reset();
}

void PCA9685ThrusterModule::Start(uint32_t now_ms)
{
    manager_.Start(now_ms);
}

void PCA9685ThrusterModule::Update(uint32_t now_ms)
{
    manager_.Update(now_ms);
}

bool PCA9685ThrusterModule::SubmitOutputs(
    const int32_t *pwm_us, size_t count, uint32_t now_ms)
{
    return manager_.SubmitOutputs(pwm_us, count, now_ms);
}

void PCA9685ThrusterModule::EmergencyNeutral(uint32_t now_ms)
{
    manager_.EmergencyNeutral(now_ms);
}

/* ---- Queries ---- */

bool PCA9685ThrusterModule::IsReady() const
{
    return manager_.IsReady();
}

bool PCA9685ThrusterModule::IsRecovering() const
{
    return manager_.IsRecovering();
}

bool PCA9685ThrusterModule::IsFaulted() const
{
    return manager_.IsFaulted();
}

ActuatorStartupState PCA9685ThrusterModule::State() const
{
    return manager_.State();
}

ActuatorFaultReason PCA9685ThrusterModule::FaultReason() const
{
    return manager_.FaultReason();
}

/* ---- Diagnostics ---- */

void PCA9685ThrusterModule::GetDiagnostics(
    PCA9685Diagnostics *diag) const
{
    driver_.GetDiagnostics(diag);
}
