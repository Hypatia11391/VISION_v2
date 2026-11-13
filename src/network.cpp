#include "network.hpp"
#include "constants.hpp"
#include "utils.hpp"
#include <vector>
//#include <chrono>
//#include <thread>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>

VS::Network::Network(VS::ThreadSafeQueue<VS::CameraPoseSet>* input_queue) : pose_queue(input_queue) {}

void VS::Network::run() {
    auto nt = nt::NetworkTableInstance::GetDefault();
    nt.StartClient4("RaspberryPiVision");
    nt.SetServerTeam(11391);
    nt.SetServer("10.113.91.2", 5810);

    auto table = nt.GetTable("datatable");
    auto pose_pub = table->GetDoubleArrayTopic("robot_pose").Publish({.periodic = Constants::net_table_update_speed/1000});

    VS::CameraPoseSet poses;

    while (true) {
        //std::cout << "Trying to pop pose" << std::endl;
        pose_queue->pop(poses);
        //std::cout << "Succeeded popping pose" << std::endl;

        for (int j = 0; j < poses.camera_poses.size(); j++) {
                // Serialize CameraPose struct into an array of doubles
                std::vector<double> pose_vector;

                pose_vector.push_back(poses.timestamp);
                pose_vector.push_back(static_cast<double>(poses.camera_id));
                
                VS::CameraPose camera_pose = poses.camera_poses[j];
                pose_vector.push_back(static_cast<double>(camera_pose.apriltag_set_number));
                pose_vector.insert(pose_vector.end(), camera_pose.pose.data(), camera_pose.pose.data() + camera_pose.pose.size());
                pose_vector.insert(pose_vector.end(), camera_pose.uncertainty.data(), camera_pose.uncertainty.data() + camera_pose.uncertainty.size());

                //std::cout << "Vals:" << std::endl;

                //for (const auto& val : pose_vector) {
                //        std::cout << val << ", ";
                //}
                //std::cout << std::endl;

                pose_pub.Set(pose_vector);
                //std::cout << "Set to network" << std::endl;
        }
    }
}
