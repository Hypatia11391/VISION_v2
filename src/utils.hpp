#pragma once

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "Eigen/Dense"
#include <opencv2/opencv.hpp>

namespace  VS {

struct Points{
    int set_id;
    std::vector<cv::Point3d> obj_points;
    std::vector<cv::Point2d> img_points;
};

struct CameraInfo {
    int cam_res_width;
    int cam_res_height;    
    int cam_FPS;
    std::vector<double> distortion_constants;
    cv::Matx33d intrinsics;
    const std::string device_path;
};

struct Image {
    cv::Mat frame;
    int camera_id;
    double timestamp;
    uint64_t frame_sequence_number;
};

struct CameraPose {
    int apriltag_set_number;
    Eigen::Matrix4d pose;
    Eigen::Matrix<double, 6, 1> uncertainty;
};

struct CameraPoseSet {
    int camera_id;
    std::vector<CameraPose> camera_poses;
    double timestamp;
};

inline Eigen::MatrixXd jacobianPsuedoInverse(const Eigen::MatrixXd &J, double lambda = 0.001) { // <--------- This is AI generated. Use at your own risk.
    int rows = J.rows(); // <------------ syntax error: J is pointer?
    int cols = J.cols();
    
    if (rows <= cols) {
        // Fat or square matrix
        Eigen::MatrixXd JJt = J * J.transpose();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(rows, rows);
        return J.transpose() * (JJt + lambda * lambda * I).inverse();
    } else {
        // Tall matrix
        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(cols, cols);
        return (JtJ + lambda * lambda * I).inverse() * J.transpose();
    }
}

inline Eigen::Matrix4d getTransform(const cv::Mat rvec, const cv::Mat tvec) {
    cv::Mat rotation_matrix;
    Eigen::Matrix4d T_out;

    // Convert the rotation vector
    cv::Rodrigues(rvec, rotation_matrix);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> eigen_rot((double*)rotation_matrix.data);
    T_out.block<3,3>(0,0) = eigen_rot;

    // Convert the translation vector
    T_out.block<3, 1>(0, 3) = Eigen::Vector3d::Map((double*)tvec.data);

    return T_out;
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
