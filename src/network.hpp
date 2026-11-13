#pragma once

#include "utils.hpp"
#include <vector>

namespace VS {
class Network {
private:
    VS::ThreadSafeQueue<CameraPoseSet>* pose_queue;
    nt::NetworkTableInstance& nt_inst;
public:
    Network(VS::ThreadSafeQueue<CameraPoseSet>* pose_queue, nt::NetworkTableInstance& nt_inst);

    void run();
};
}