#include <thread>
#include <vector>

#include "network.hpp"
#include "utils.hpp"

#include "video_stream.hpp"
#include "pose_estimation.hpp"

int main () {
    // Set up queues for camera 0
    VS::ThreadSafeQueue<VS::Image> image_queue_0;
    VS::ThreadSafeQueue<VS::CameraPoseSet> pose_queue_0;

    // Set up video streaming and pose estimator classes for camera 0
    VS::VideoStream stream_0{0, image_queue_0}; // <------------------------- ToDo, check device path
    VS::PoseEstimator estimator_0{0, image_queue_0, pose_queue_0};

    // Start stream and pose estimnaton threads for camera 0
    std::cout << "[INFO] Starting video stream 0 thread" << std::endl;
    std::thread t_stream_0(&VS::VideoStream::video_stream, &stream_0);
    std::cout << "[INFO] Starting pose estimator 0 thread" << std::endl;
    std::thread t_estimator_0(&VS::PoseEstimator::run, &estimator_0);

    // Set up queues for camera 1
    VS::ThreadSafeQueue<VS::Image> image_queue_1;
    VS::ThreadSafeQueue<VS::CameraPoseSet> pose_queue_1;

    // Set up video streaming and pose estimator classes for camera 1
    VS::VideoStream stream_1{1, image_queue_1}; // <------------------------- ToDo, check device path
    VS::PoseEstimator estimator_1{1, image_queue_1, pose_queue_1};

    // Start stream and pose estimnaton threads for camera 1
    std::cout << "[INFO] Starting video stream 1 thread" << std::endl;
    std::thread t_stream_1(&VS::VideoStream::video_stream, &stream_1);
    std::cout << "[INFO] Starting pose estimator 1 thread" << std::endl;
    std::thread t_estimator_1(&VS::PoseEstimator::run, &estimator_1);

    // Create networking thread.
    std::vector<VS::ThreadSafeQueue<VS::CameraPoseSet>> pose_queues;
    pose_queues.push_back(pose_queue_0);
    pose_queues.push_back(pose_queue_1);
    
    VS::Network network_obj(pose_queues);
    std::cout << "[INFO] Starting networking thread" << std::endl;
    std::thread t_network(&VS::Network::run, network_obj);

    // Make sure to wait for threads to finish
    t_stream_0.join();
    t_estimator_0.join();
    t_stream_1.join();
    t_estimator_1.join();
    t_network.join();

    return 0;
}