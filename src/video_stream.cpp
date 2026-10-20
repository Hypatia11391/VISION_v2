#include <opencv2/opencv.hpp>

#include <iostream>
#include <opencv2/videoio.hpp>
#include <vector>
#include <numeric>
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

    using clk = std::chrono::steady_clock;
    auto ms = [](auto d){ return std::chrono::duration<double, std::milli>(d).count(); };

    // Calc time offset between monotomic and systemclock
    auto rt1 = std::chrono::system_clock::now();
    auto mono = clk::now(); // Note: steady clock is the monotomic
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

    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); 
    double exposureValue = 50.0; 
    bool success = cap.set(cv::CAP_PROP_EXPOSURE, exposureValue);
    if (success) {
        std::cout << "Exposure successfully set to: " << exposureValue/10 << "ms" << std::endl;
    } else {
        std::cerr << "Warning: Failed to set exposure. Property might not be supported by this camera/driver." << std::endl;
    }

    uint64_t frame_count = 0;
    cv::Mat current_frame;
    double capture_time_monotomic;

    // Diagnostics
    uint64_t last_diagnostic_frame = 0;
    double fps_ms_counter_init = duration_cast<std::chrono::duration<double, std::milli>>(clk::now().time_since_epoch()).count();
    std::vector<double> frame_ages;

    // Continuous capture loop
    while (true) {
        // FPS diagnostic
        double fps_ms_counter_current = duration_cast<std::chrono::duration<double, std::milli>>(clk::now().time_since_epoch()).count();
        double ms_since_last_sample = fps_ms_counter_current - fps_ms_counter_init;
        if (ms_since_last_sample > 1000) {
            double fps = 1000*(frame_count - last_diagnostic_frame)/ms_since_last_sample;
            std::cout << "Currently running at " << fps << "fps" << std::endl;

            if (!frame_ages.empty()) {
                double avg_frame_age = std::accumulate(frame_ages.begin(), frame_ages.end(), 0.0)/frame_ages.size();
                std::cout << "Avg frame age: " << avg_frame_age << "ms\n";
            }

            fps_ms_counter_init = fps_ms_counter_current;
            last_diagnostic_frame = frame_count;
        }

        auto t0 = clk::now();
        if (cap.grab()) {
            auto t1 = clk::now();
            double grab_ms = ms(t1.time_since_epoch()) - ms(t0.time_since_epoch());
            std:: cout << "Grabbed frame in: " << grab_ms << "ms\n";
            
            capture_time_monotomic = cap.get(cv::CAP_PROP_POS_MSEC);
            double now_mono_ms = std::chrono::duration<double, std::milli>(clk::now().time_since_epoch()).count();
            double age_ms = now_mono_ms - capture_time_monotomic;  // log this
            frame_ages.push_back(age_ms);
        }
        else {
            std::cerr << "Warning: Dropped frame number " << frame_count << " on camera " << cam_id << std::endl;
            frame_count++;
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