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
    int cam_id;

    VS::ThreadSafeQueue<Image>& frame_queue;
    VS::ThreadSafeQueue<CameraPoseSet>& output_pose_queue;

    // Returns objectpoints and image points in the form of an array of len 2. Obj and Img pointst are each a vector one element
    // for each relevant ref frame, and the image and opject points are contained as a vector of points in each of these.
    VS::points get_points(zarray_t *detections, int set_id);

    // Takes in a points object, and outputs a pose transform and covariance matrix
    std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 6, 1>> estimate_pose(points points);

    // Takes in a set of inliers and outputs the covariance matrix
    Eigen::Matrix<double, 6, 1> get_stds(points inliers, cv::Mat rvec, cv::Mat tvec);// Extra function, compute covariance.


public:
    void run(); // Creates the thread to process detections and runs private functions to produce poses. Writes poses to buffer

    PoseEstimator(int id, VS::ThreadSafeQueue<Image>& input_queue, VS::ThreadSafeQueue<Image>& output_queue);
    
    ~PoseEstimator();
};
}