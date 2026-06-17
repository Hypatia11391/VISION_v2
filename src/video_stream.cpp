#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>
#include "utils.hpp"

#include "constants.hpp"
#include "video_stream.hpp"

VS::VideoStream::VideoStream(int id, VS::ThreadSafeQueue<Image>& queue, const std::string& device_path)
    : cam_id(id), output_queue(queue), device_path(device_path) {}

void VS::VideoStream::video_stream() {
    // Initialize camera using the persistent V4L2 path
    cv::VideoCapture cap(device_path, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        std::cout << "Error: Could not open camera " << cam_id << " at " << device_path << std::endl;
        return;
    }

    // Hardware configurations.
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // <------ try YUV2. That might be bad, or might work better.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, Constants::cameras[cam_id-1].cam_res_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, Constants::cameras[cam_id-1].cam_res_height);
    cap.set(cv::CAP_PROP_FPS, Constants::cameras[cam_id-1].cam_FPS);

    uint64_t frame_count = 0;
    cv::Mat current_frame;

    // Continuous capture loop
    while (true) {
        double capture_time;

        if (cap.grab()) {
            double capture_time = cap.get(cv::CAP_PROP_POS_MSEC);
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
            image.timestamp = capture_time;
            image.frame_sequence_number = frame_count;

            // Push to the compute thread
            output_queue.push(image);
        }
    }
}
}