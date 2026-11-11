#pragma once

#include "utils.hpp"
#include <vector>

namespace VS {
class Network {
private:
    VS::ThreadSafeQueue<CameraPoseSet>* pose_queue;
public:
    Network(VS::ThreadSafeQueue<CameraPoseSet>* pose_queue);

    void run();
};
}