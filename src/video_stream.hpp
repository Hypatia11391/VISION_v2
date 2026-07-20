#pragma once

#include <opencv2/opencv.hpp>
#include "utils.hpp"

namespace VS {
class VideoStream {
private:
    int cam_id;
    VS::ThreadSafeQueue<Image>& output_queue;

public:
    VideoStream(int cam_id, VS::ThreadSafeQueue<Image>& output_queue);

    void video_stream();

};
}