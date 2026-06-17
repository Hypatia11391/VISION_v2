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
    inline constexpr std::array<std::vector<int>, num_ref_frames> apriltag_ids_for_each_frame = {
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32},
        {2, 3, 4, 5, 8, 9, 10, 11},
        {18, 19, 20, 21, 24, 25, 26, 27}
    };

    inline constexpr std::vector<VS::CameraInfo> cameras = {
        // --- CAMERA 1 ---
        {
            1600, 1304, 60, // int cam_res_width, int cam_res_height, int cam_FPS
            {0, 0, 0, 0, 0}, // std::vector<double> distortion_constants
            cv::Matx33d(968.98165733, 0.0        , 683.85193417,// cv::Matx33d intrinsics <------------------------ PLACEHOLDERS
                        0.0         , 969.8771812, 519.93781744,
                        0.0         , 0.0        , 1.0          )
        }
        
        // --- CAMERA 2 ---
        {
            1600, 1304, 60, // int cam_res_width, int cam_res_height, int cam_FPS
            {0, 0, 0, 0, 0}, // std::vector<double> distortion_constants
            cv::Matx33d(968.98165733, 0.0        , 683.85193417,// cv::Matx33d intrinsics <------------------------ PLACEHOLDERS
                        0.0         , 969.8771812, 519.93781744,
                        0.0         , 0.0        , 1.0          )
        }
    };
}