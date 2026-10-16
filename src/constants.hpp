#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "utils.hpp"

namespace Constants {
    inline constexpr double tag_size = 0.1651; // Tag side length in meters
    
    // This is the number of reference frames to calculate the pose in.
    // Typically 1 global, + 1 for every important indipendant gamepice.
    // Eg. 3 for REBUILT 2026 (global, +1 for each hub).
    inline constexpr int num_ref_frames = 3;

    // These are the sets of tag ids relevant for each reference frame. THEY MUST BE IN SORTED INCREASING ORDER!!!!
    inline const std::array<std::vector<int>, num_ref_frames> apriltag_ids_for_each_frame = {{
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32},
        {2, 3, 4, 5, 8, 9, 10, 11},
        {18, 19, 20, 21, 24, 25, 26, 27}
    }};

    inline const std::vector<VS::CameraInfo> cameras = {
        // --- CAMERA 1 --- This is the camera with the black lens clamp
        {
            1600, 1304, 60, // int cam_res_width, int cam_res_height, int cam_FPS
            {-0.01998613, 0.01144283, -0.00374616, -0.00283593, -0.02289775}, // std::vector<double> distortion_constants
            cv::Matx33d(1373.92138, 0.0        ,   775.841832,// cv::Matx33d intrinsics <------------------------ PLACEHOLDERS
                                    0.0         , 1378.46488,  644.592015,
                                    0.0         , 0.0        , 1.0          ),
            "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0" // <---------------------------------This may be wrong ------------------------- ToDo Double check device path
        },
        
        // --- CAMERA 2 --- This is the camera with the purple lens clamp
        {
            1600, 1304, 60, // int cam_res_width, int cam_res_height, int cam_FPS
            {0, 0, 0, 0, 0}, // std::vector<double> distortion_constants
            cv::Matx33d(1373.92138, 0.0        ,   775.841832,// cv::Matx33d intrinsics <------------------------ PLACEHOLDERS
                                    0.0         , 1378.46488,  644.592015,
                                    0.0         , 0.0        , 1.0          ),
            "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0" // <---------------------------------This may be wrong ------------------------- ToDo Double check device path
        }
    };

    inline constexpr double quad_decimate = 2.0;
    inline constexpr double quad_sigma = 0.0;
    inline constexpr int nthreads = 2;
    inline constexpr int refine_edges = 1;
    
    inline constexpr int iterations = 100; // Number of iterations for the PnP algorithm to run
    inline constexpr double reprojection_error = 8.0;// reprojectionError (threshold to consider a point an inlier)
    inline constexpr double req_confidence = 0.99; // Required confidence for the pnp algorithm. Increasing requires more iterations.

    inline constexpr double pixel_sigma_sq = 1.0; // The estimated squared standard deviation in image point [pixels] accuracy
}
