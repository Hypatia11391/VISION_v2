#pragma once

#include <opencv2/opencv.hpp>
#include "utils.hpp"

namespace VS {
class VideoStream {
private:
    int cam_id;
    VS::ThreadSafeQueue<Image>& output_queue;
    const std::string device_path;

public:
    VideoStream(int cam_id, VS::ThreadSafeQueue<Image>& output_queue, const std::string device_path);

    void video_stream();

};
}