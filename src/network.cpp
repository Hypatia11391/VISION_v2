#include "network.hpp"
#include "utils.hpp"
#include <vector>

VS::Network::Network(std::vector<VS::ThreadSafeQueue<VS::CameraPoseSet>> input_queues) : pose_queues(input_queues) {}

void VS::Network::run() {
    int num_cams = pose_queues.size();
    std::vector<VS::CameraPoseSet> poses;
    poses.resize(num_cams);

    while (true) {
        for (int i = 0; i < num_cams; i ++) {
            pose_queues[i].pop(poses[i]);
        }

        // Stuff with poses happens here.
    }
}