#pragma once
#include "include/protocol.h"
#include "DataBus.hpp"

class Communication {
public:
    void Init()   { Comm_Init(); }
    void Update() { Comm_Process(); }
};
