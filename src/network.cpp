#include "network.hpp"
#include "utils.hpp"
#include <vector>
#include <chrono>
#include <thread>

VS::Network::Network(std::vector<VS::ThreadSafeQueue<VS::CameraPoseSet>> input_queues) : pose_queues(input_queues) {}

void VS::Network::run() {
    auto nt = nt::NetworkTableInstance::GetDefault();
    nt.StartClient4("RaspberryPiVision");
    nt.SetServer("192.168.0.44", 5810);

    auto table = nt.GetTable("Vision");
    auto pose_pub = table->GetDoubleArrayTopic("pose").Publish();
    
    int num_cams = pose_queues.size();
    std::vector<VS::CameraPoseSet> poses;
    poses.resize(num_cams);

    while (true) {
        for (int i = 0; i < num_cams; i ++) {
            pose_queues[i].pop(poses[i]);

            for (int j = 0; j < poses[i].camera_poses.size(); j ++) {
                // Serialize CameraPose struct into an array of doubles
                std::vector<double> pose_vector;
                pose_vector.push_back(poses[i].timestamp);
                pose_vector.push_back(static_cast<double>(poses[i].camera_id))
                
                camera_pose = poses[i].camera_poses[j];
                pose_vector.push_back(static_cast<double>(camera_pose.apriltag_set_number));
                pose_vector.insert(pose_vector.end(), camera_pose.pose.data(), camera_pose.pose.data()+camera_pose.pose.size());
                pose_vector.insert(pose_vector.end(), camera_pose.uncertainty.data(), camera_pose.uncertainty.data()+camera_pose.uncertainty.size());

                pose_pub.Set(pose_vector);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Sleep between pose checks. TODO delay may need to be adjusted or removed depending on how fast new pose data is being added to pose_queues
    }
}

void 
