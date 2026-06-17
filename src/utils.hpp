#pragma once

#include "constants.hpp"
#include <chrono>
#include <string>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "constants.hpp"
#include <opencv2/opencv.hpp>

namespace  VS {

struct points{
    bool exists;
    std::vector<cv::point3f> obj_points;
    std::vector<cv::point2f> img_points;
}

struct CameraInfo {
    int cam_res_width;
    int cam_res_height;    
    int cam_FPS;
    std::vector<double> distortion_constants;
    cv::Matx33d intrinsics;
}

struct Image {
    cv::Mat frame;
    int camera_id;
    double timestamp;
    uint64_t frame_sequence_number;
};

struct CameraPose {
    Eigen::Matrix4d pose;
    double translation_err;
    double rotation_err;
};

struct CameraPoseSet {
    int camera_id;
    std::vector<CameraPose> camera_poses;
    double timestamp;
}

template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    size_t max_size_;

public:
    // Set a small max_size to prevent the queue from growing infinitely if computation falls behind capture.
    ThreadSafeQueue(size_t max_size = 2) : max_size_(max_size) {}

    // Called by the producer thread
    void push(T item) {
        // std::lock_guard automatically locks the mutex, and unlocks it when this function finishes. Never manually lock/unlock!
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Drop the oldest frame if falling behind.
        if (queue_.size() >= max_size_) {
            queue_.pop(); 
        }
        
        queue_.push(std::move(item));
        
        // Wake up ONE sleeping consumer thread that is waiting on this queue.
        cond_var_.notify_one(); 
    }

    // Called by the consumer thread)
    void pop(T& item) {
        // std::unique_lock is required for condition variables.
        std::unique_lock<std::mutex> lock(mutex_);
        
        // This puts the thread to sleep until the queue is not empty. It safely releases the mutex while sleeping, and reacquires it upon waking.
        cond_var_.wait(lock, [this]() { return !queue_.empty(); });
        
        item = std::move(queue_.front());
        queue_.pop();
        
        // The lock is automatically released when this function ends.
    }
};
}