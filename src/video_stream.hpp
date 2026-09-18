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

    double compute_realtime_monotonic_offset_ms();
    void video_stream();

};
}