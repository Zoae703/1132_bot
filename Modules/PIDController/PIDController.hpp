#pragma once
#include "include/PID.h"
// Thin wrapper — PID is a utility, Init/Update are no-ops or Reset.
class PIDController {
public:
    PIDController() {}
    void Init() {}
    void Update() {}
};
