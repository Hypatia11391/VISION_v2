#pragma once

#include "utils.hpp"
#include <vector>

namespace VS {
class Network {
private:
    std::vector<VS::ThreadSafeQueue<CameraPoseSet>> pose_queues;
public:
    Network(std::vector<VS::ThreadSafeQueue<CameraPoseSet>> input_queues);

    void run();
};
}