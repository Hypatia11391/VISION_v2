#include <opencv2/opencv.hpp>

#include <iostream>
#include "utils.hpp"

#include "constants.hpp"
#include "video_stream.hpp"

VS::VideoStream::VideoStream(int id, VS::ThreadSafeQueue<Image>& queue)
    : cam_id(id), output_queue(queue) {}

void VS::VideoStream::video_stream() {
    // Initialize camera using the persistent V4L2 path
    cv::VideoCapture cap(Constants::cameras[cam_id].device_path, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        std::cout << "Error: Could not open camera " << cam_id << " at " << Constants::cameras[cam_id].device_path << std::endl;
        return;
    }

    // Calc time offset between monotomic and systemclock
    auto rt1 = std::chrono::system_clock::now();
    auto mono = std::chrono::steady_clock::now(); // Note: steady clock is the monotomic
    auto rt2 = std::chrono::system_clock::now();
    double rt1_ms = duration_cast<std::chrono::duration<double, std::milli>>(rt1.time_since_epoch()).count();
    double rt2_ms = duration_cast<std::chrono::duration<double, std::milli>>(rt2.time_since_epoch()).count();
    double mono_ms = duration_cast<std::chrono::duration<double, std::milli>>(mono.time_since_epoch()).count();
    double rt_mid_ms = (rt1_ms + rt2_ms) / 2.0;
    double offset = rt_mid_ms - mono_ms;

    // Hardware configurations.
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // <------ try YUV2. That might be bad, or might work better.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, Constants::cameras[cam_id].cam_res_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, Constants::cameras[cam_id].cam_res_height);
    cap.set(cv::CAP_PROP_FPS, Constants::cameras[cam_id].cam_FPS);

    uint64_t frame_count = 0;
    cv::Mat current_frame;
    double capture_time_monotomic;

    // Continuous capture loop
    while (true) {

        if (cap.grab()) {
            capture_time_monotomic = cap.get(cap.get(cv::CAP_PROP_POS_MSEC));
        }
        else {
            std::cerr << "Warning: Dropped frame number " << frame_count << " on camera " << cam_id << std::endl;
            continue;
        }

        if (cap.retrieve(current_frame)) {
            frame_count++;

            // Package the data and metadata
            Image image;
            image.frame = current_frame.clone();
            image.camera_id = cam_id;
            image.timestamp = capture_time_monotomic + offset;
            image.frame_sequence_number = frame_count;

            // Push to the compute thread
            output_queue.push(image);
        }
    }
}