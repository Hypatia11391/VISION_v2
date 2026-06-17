#pragma once

// Standard C++
//#include <vector>

// Libraries
#include <apriltag/apriltag_pose.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/zarray.h>
#include <opencv2/opencv.hpp>
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
    std::array<points, Constants::num_ref_frames> get_points(zarray_t *detections);

public:
    void run(); // Creates the thread to process detections and runs private functions to produce poses. Writes poses to buffer

    PoseEstimator(int id, VS::ThreadSafeQueue<Image>& input_queue, VS::ThreadSafeQueue<Image>& output_queue);
    
    Eigen::Matrix4d estimate_pose(points& points);

    ~PoseEstimator();
};
}