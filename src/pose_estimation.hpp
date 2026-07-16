#pragma once

// Standard C++
//#include <vector>

// Libraries
#include <Eigen/src/Core/Matrix.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/zarray.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Eigen/Dense"

// Custom Headers
#include "constants.hpp"
#include "utils.hpp"

namespace VS {
class PoseEstimator {
private:
    // Local variables
    int cam_id;
    VS::ThreadSafeQueue<Image>& frame_queue;
    VS::ThreadSafeQueue<CameraPoseSet>& output_pose_queue;

    // Returns the object and imag points of the detections in the image, that are in the apritag set defined by Constants::apriltag_ids_for_each_frame[set_id].
    std::vector<VS::Points> get_points(zarray_t *detections);

    // Finds the uncertainty of the estimated pose. Outputs the 6 DOF standard deviations.
    Eigen::Matrix<double, 6, 1> get_stds(VS::Points actual_points, cv::Mat inliers, cv::Mat rvec, cv::Mat tvec);// Extra function, compute covariance.

    // Takes in a points object, and outputs a pose transform and covariance matrix
    VS::CameraPoseSet estimate_pose(std::vector<VS::Points> points);

public:
    void run(); // Creates the thread to process detections and runs private functions to produce poses. Writes poses to buffer

    PoseEstimator(int id, VS::ThreadSafeQueue<Image>& input_queue, VS::ThreadSafeQueue<CameraPoseSet>& output_queue);
};
}