#include "network.hpp"
#include "constants.hpp"
#include "utils.hpp"
#include <vector>
#include <chrono>
#include <thread>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>

VS::Network::Network(std::vector<VS::ThreadSafeQueue<VS::CameraPoseSet>*> input_queues) : pose_queues(input_queues) {}

void VS::Network::run() {
    auto nt = nt::NetworkTableInstance::GetDefault();
    nt.StartClient4("RaspberryPiVision");
    nt.SetServerTeam(11391);
    nt.SetServer("10.113.91.2", 5810);

    auto table = nt.GetTable("datatable");
    auto pose_pub = table->GetDoubleArrayTopic("robot_pose").Publish({.periodic = Constants::net_table_update_speed});
    
    int num_cams = pose_queues.size();
    std::vector<VS::CameraPoseSet> poses;
    poses.resize(num_cams);

    while (true) {
	std::cout << "num_cams = " << num_cams << std::endl;

        for (int i = 0; i < num_cams; i++) {
            std::cout << "Trying to pop pose queues" << std::endl;
	    pose_queues[i]->pop(poses[i]);

	    std::cout << "pose count = " << poses[i].camera_poses.size() << ", with i = " << i << std::endl;

            for (int j = 0; j < poses[i].camera_poses.size(); j++) { // <---------------------- Might fail because num poses is > than num cameras
                // Serialize CameraPose struct into an array of doubles
                std::vector<double> pose_vector;
                pose_vector.push_back(poses[i].timestamp);
                pose_vector.push_back(static_cast<double>(poses[i].camera_id));
                
                VS::CameraPose camera_pose = poses[i].camera_poses[j];
                pose_vector.push_back(static_cast<double>(camera_pose.apriltag_set_number));
                pose_vector.insert(pose_vector.end(), camera_pose.pose.data(), camera_pose.pose.data() + camera_pose.pose.size());
                pose_vector.insert(pose_vector.end(), camera_pose.uncertainty.data(), camera_pose.uncertainty.data() + camera_pose.uncertainty.size());

		std::cout << "Vals:" << std::endl;

		for (const auto& val : pose_vector) {
			std::cout << val << ", ";
		}
		std::cout << std::endl;

                pose_pub.Set(pose_vector);
		std::cout << "Set to network" << std::endl;
            }
	    std::cout << "ended j loop with i = " << i << " and num_cams = " << num_cams << std::endl;
        }
	std::cout << "Trying to sleep thread" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep between pose checks. TODO delay may need to be adjusted or removed depending on how fast new pose data is being added to pose_queues
	std::cout << "slept thread" << std::endl;
    }

    std::cout << "SOMEHOW EXITED WHLE TRUE LOOP HOW???????????????????????????????????????????????????\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
}
