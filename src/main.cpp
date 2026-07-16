#include <vector>
#include <thread>

#include "utils.hpp"
#include "constants.hpp"

#include "video_stream.hpp"
#include "pose_estimation.hpp"

int main () {
    // Set up queues for camera 1
    VS::ThreadSafeQueue<VS::Image> image_queue_1;
    VS::ThreadSafeQueue<VS::CameraPoseSet> pose_queue_1;

    // Set up video streaming and pose estimator classes for camera 1
    VS::VideoStream stream_1{1, image_queue_1, "ToDo - Add camera path"};
    VS::PoseEstimator estimator_1{1, image_queue_1, pose_queue_1};

    // Start stream and pose estimnaton threads for camera 1
    std::thread t_stream_1(&VS::VideoStream::video_stream, &stream_1);
    std::thread t_estimator_1(&VS::PoseEstimator::run, &estimator_1);

    // Set up queues for camera 2
    VS::ThreadSafeQueue<VS::Image> image_queue_2;
    VS::ThreadSafeQueue<VS::CameraPoseSet> pose_queue_2;

    // Set up video streaming and pose estimator classes for camera 2
    VS::VideoStream stream_2{2, image_queue_2, "ToDo - Add camera path"};
    VS::PoseEstimator estimator_2{2, image_queue_2, pose_queue_2};

    // Start stream and pose estimnaton threads for camera 2
    std::thread t_stream_2(&VS::VideoStream::video_stream, &stream_2);
    std::thread t_estimator_2(&VS::PoseEstimator::run, &estimator_2);

    // Create networking thread.


    // Make sure to wait for threads to finish
    t_stream_1.join();
    t_estimator_1.join();
    t_stream_2.join();
    t_estimator_2.join();

    return 0;
}