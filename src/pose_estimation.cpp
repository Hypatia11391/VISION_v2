#include "Eigen/Dense"
#include <algorithm>

#include <apriltag/apriltag_pose.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/zarray.h>

#include "pose_estimation.hpp"
#include "apriltag_locs.hpp"
#include "constants.hpp"
#include "utils.hpp"
#include <opencv2/core/eigen.hpp>
#include <vector>

VS::PoseEstimator::PoseEstimator(int id, VS::ThreadSafeQueue<Image>& input_queue, VS::ThreadSafeQueue<CameraPoseSet>& output_queue)
    : cam_id(id), frame_queue(input_queue), output_pose_queue(output_queue) {}

std::vector<VS::Points> VS::PoseEstimator::get_points(zarray_t *detections){ // <------------------------- Test point order. May be incorrect.
    std::array<Eigen::Vector4d, 4> obj_point_choices;
    obj_point_choices[0] = Eigen::Vector4d(-Constants::tag_size/2, Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[1] = Eigen::Vector4d(Constants::tag_size/2, Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[2] = Eigen::Vector4d(Constants::tag_size/2, -Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[3] = Eigen::Vector4d(-Constants::tag_size/2, -Constants::tag_size/2, 0.0, 1.0);

    std::vector<VS::Points> out;

    for (int set_id = 0; set_id < Constants::num_ref_frames; set_id ++) {
        VS::Points points;
        points.set_id = set_id;

        // For each detection
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // Check if the detection is in the set
            if (std::binary_search(Constants::apriltag_ids_for_each_frame[set_id].begin(), Constants::apriltag_ids_for_each_frame[set_id].end(), det->id)) {
                for (int corner = 0; corner < 4; corner ++) { // <----------------------------- Possible optimizations: Memory alocation of vector, and pose inversion to custom closed form function, or transpositiona rather than inversion.
                    Eigen::Vector4d obj_point_eigen = Constants::apriltag_poses_in_global[det->id - 1].inverse() * obj_point_choices[corner];
                    
                    cv::Point3d obj_point = {obj_point_eigen[0], obj_point_eigen[1], obj_point_eigen[2]};
                    cv::Point2d im_point = {det->p[corner][0], det->p[corner][1]};

                    points.obj_points.push_back(obj_point);
                    points.img_points.push_back(im_point);
                }
            }
        }

        out.push_back(points);
    }

    return out;
}

Eigen::Matrix<double, 6, 1> VS::PoseEstimator::get_stds(VS::Points actual_points, cv::Mat inliers, cv::Mat rvec, cv::Mat tvec){
    const int num_inliers = inliers.rows;

    // Define the inlier points
    VS::Points inlier_points;
    inlier_points.img_points.reserve(num_inliers);
    inlier_points.obj_points.reserve(num_inliers);

    std::cout <<"[DEBUG] initialized inlier points. About to fill." << std::endl;

    for (int inlier = 0; inlier < num_inliers; inlier ++) {
        int inlier_idx = inliers.at<int>(inlier, 0);

        (inlier_points.img_points)[inlier] = (actual_points.img_points)[inlier_idx];
        (inlier_points.obj_points)[inlier] = (actual_points.obj_points)[inlier_idx];
    }

    std::cout << "[DEBUG] Size of inlier_points: " << inlier_points.img_points.size() << std::endl;

    std::vector<cv::Point2d> reprojected_points;
    cv::Mat jacobian;

    Eigen::MatrixXd delta_im_pts(2*num_inliers, 1); // <---------------------------- Wrong shape? Transpose?
    Eigen::MatrixXd standard_divs(num_inliers, 6);

    cv::projectPoints(inlier_points.obj_points,
                      rvec,
                      tvec,
                      Constants::cameras[cam_id].intrinsics,
                      Constants::cameras[cam_id].distortion_constants,
                      reprojected_points,
                      jacobian
    );

    std::cout << "[DEBUG] Size of img_points: " << actual_points.img_points.size() << std::endl;
    std::cout << "[DEBUG] Size of projected_points: " << reprojected_points.size() << std::endl;

    std::vector<cv::Point2d> delta_im_pts_cv;
    std::transform(reprojected_points.begin(), reprojected_points.end(), (inlier_points.img_points).begin(), delta_im_pts_cv.begin(), std::minus<cv::Point2d>());

    for (int point = 0; point < num_inliers; point++) {
        delta_im_pts(2*point, 0) = delta_im_pts_cv[point].x;
        delta_im_pts(2*point + 1, 0) = delta_im_pts_cv[point].y;
    }

    Eigen::MatrixXd eigen_jacobian;
    cv::cv2eigen(jacobian, eigen_jacobian);

    // take the inverse of the jacobian, then multiply by delta im points to get delta pose.
    Eigen::MatrixXd J_inv(6, 2*num_inliers);
    J_inv << VS::jacobianPsuedoInverse(eigen_jacobian);
    Eigen::Matrix<double, 6, 1> delta_pose = J_inv * delta_im_pts;

    return delta_pose;
}

VS::CameraPoseSet VS::PoseEstimator::estimate_pose(std::vector<VS::Points> points) {
    VS::CameraPoseSet out;
    out.camera_id = cam_id;
    
    for (const auto& points_in_ref_frame : points) {
        if (points_in_ref_frame.obj_points.size() < 4) {
            continue;
        }

        VS::CameraPose pose;

        cv::Mat rvec;
        cv::Mat tvec;
        cv::Mat inliers;

        bool successfulPnP = cv::solvePnPRansac(points_in_ref_frame.obj_points,
                                                points_in_ref_frame.img_points,
                                                Constants::cameras[cam_id].intrinsics,
                                                Constants::cameras[cam_id].distortion_constants,
                                                rvec,
                                                tvec,
                                                false, // useExtrinsicGuess <------------ Coulb be true with an estimation of current pose as just the last frame. Try if needed for time optimization.
                                                Constants::iterations,
                                                Constants::reprojection_error,
                                                Constants::req_confidence,
                                                inliers,
                                                cv::SOLVEPNP_ITERATIVE
        );

        if (!successfulPnP) {
            continue;
        }

        // Convert output Rodrigues and tvec to 4x4 transform matrix
        pose.pose = VS::getTransform(rvec, tvec);

        // Get pose standard deviations
        pose.uncertainty = get_stds(points_in_ref_frame, inliers, rvec, tvec);

        pose.apriltag_set_number = points_in_ref_frame.set_id;

        out.camera_poses.push_back(pose);
    }

    return out;

}

void VS::PoseEstimator::run() {
    VS::Image frame;

    cv::Mat& frame_data = frame.frame;
    cv::Mat gray_frame;

    VS::CameraPoseSet current_pose;
    std::vector<VS::Points> points;

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Apriltag detector constants <------------------------------------- edit these?
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 2;
    td->refine_edges = 1;

    while (true) {
        frame_queue.pop(frame);

        cv::cvtColor(frame.frame, gray_frame, cv::COLOR_BGR2GRAY);

        image_u8_t im{
            .width = gray_frame.cols,
            .height = gray_frame.rows,
            .stride = (int)gray_frame.step, 
            .buf = gray_frame.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im); // <------------- reference image?
        
        if (zarray_size(detections) > 0) {
            points = get_points(detections);
            current_pose = estimate_pose(points);
            std::cout << current_pose.camera_poses[0].pose << std::endl;

            output_pose_queue.push(current_pose);
        }

        zarray_destroy(detections);
    };

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}