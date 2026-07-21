#pragma once
#include "include/protocol.h"
#include "DataBus.hpp"

class Communication {
public:
    bool Init() { return Comm_Init(); }
    void Update() { /* Binary parsing is owned by CommunicationTask. */ }
};
