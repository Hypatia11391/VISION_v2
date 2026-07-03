#include "Eigen/Dense"
#include <apriltag/apriltag_pose.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/zarray.h>

#include "pose_estimation.hpp"
#include "apriltag_locs.hpp"
#include "constants.hpp"
#include "utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <apriltag/common/zarray.h>

VS::PoseEstimator::PoseEstimator(int id, VS::ThreadSafeQueue<Image>& input_queue, VS::ThreadSafeQueue<CameraPoseSet>& output_queue)
    : cam_id(id), frame_queue(input_queue), output_pose_queue(output_queue) {}

VS::points VS::PoseEstimator::get_points(zarray_t *detections, int set_id){
    std::array<Eigen::Vector4d, 4> obj_point_choices;
    obj_point_choices[0] = Eigen::Vector4d(-Constants::tag_size/2, Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[1] = Eigen::Vector4d(Constants::tag_size/2, Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[2] = Eigen::Vector4d(Constants::tag_size/2, -Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[3] = Eigen::Vector4d(-Constants::tag_size/2, -Constants::tag_size/2, 0.0, 1.0);

    VS::points out;
    out.set_id = set_id;

    // For each detection
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // Check if the detection is in the set
        if (std::binary_search(Constants::apriltag_ids_for_each_frame[set_id].begin(), Constants::apriltag_ids_for_each_frame[set_id].end(), det->id)) {
            for (int corner = 0; corner < 4; corner ++) { // <----------------------------- Possible optimizations: Memory alocation of vector, and pose inversion to custom closed form function, or transpositiona rather than inversion.
                Eigen::Vector4d obj_point_eigen = Constants::apriltag_poses_in_global[det->id - 1].inverse() * obj_point_choices[corner];
                
                cv::point3d obj_point = {obj_point_eigen[0], obj_point_eigen[1], obj_point_eigen[2]};
                cv::point2f im_point = {det->p[corner][0], det->p[corner][1]};

                out.obj_points.push_back(obj_point);
                out.img_points.push_back(im_point);
            }
        }
    }

    return out;
}

Eigen::Matrix<double, 6, 1> VS::PoseEstimator::get_stds(VS::points actual_points, cv::Mat inliers, cv::Mat rvec, cv::Mat tvec){ // <---------------- ToDo fix major errors.
    const int num_inliers = inliers.rows;

    // Define the inlier points
    VS::points inlier_points;
    inlier_points.img_points.reserve(num_inliers);
    inlier_points.obj_points.reserve(num_inliers);

    for (int inlier = 0; inlier < num_inliers; inlier ++) {
        int inlier_idx = inliers.at<float>(inlier, 0);

        (*inlier_points.img_points)[inlier] = (*actual_points.img_points)[inlier_idx];
        (*inlier_points.obj_points)[inlier] = (*actual_points.obj_points)[inlier_idx];
    }

    std::vector<cv::Point2f> reprojected_points;
    cv::Mat jacobian;

    Eigen::Matrix<double, 2*num_inliers, 1> delta_im_pts; // <---------------------------- Wrong shape? Transpose?
    Eigen::Matrix<double, num_inliers, 6> standard_divs;

    cv::projectPoints(*inliers.obj_points,
                      rvec,
                      tvec,
                      Constants::cameras[cam_id].intrinsics,
                      Constants::cameras[cam_id].distortion_constants,
                      reprojected_points,
                      jacobian
    );

    std::vector<cv::Point2f> delta_im_pts_cv = reprojected_points - (*inliers.img_points);

    for (int point = 0; point < num_inliers; point++) {
        delta_im_pts(2*point, 0) = delta_im_pts_cv[point][0];
        delta_im_pts(2*point + 1, 0) = delta_im_pts_cv[point][1];
    }

    Eigen::MatrixXd eigen_jacobian;
    cv::cv2eigen(jacobian, eigen_jacobian);

    // take the inverse of the jacobian, then multiply by delta im points to get delta pose.
    Eigen::Matrix<double, 6, 2*num_inliers> J_inv = VS::jacobianPsuedoInverse(eigen_jacobian);
    Eigen::Matrix<double, 6, 1> delta_pose = J_inv * delta_im_pts;

    return delta_pose;
}

VS::CameraPose VS::PoseEstimator::estimate_pose(VS::points points) {
    VS::CameraPose out;

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;

    bool successfulPnP = cv::solvePnPRansac(*points.obj_points,
                                            *points.img_points,
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

    // Convert output Rodrigues and tvec to 4x4 transform matrix
    out.pose = VS::getTransform(rvec, tvec);

    // Get pose standard deviations
    out.uncertainty = get_stds(points, inliers, rvec, tvec);

    return out;

}

void VS::PoseEstimator::run() {
    VS::Image frame;
    apriltag_detector_t *td_;
    cv::Mat& frame_data;


    while (true) {
        frame_queue.pop(frame);
        frame_data = frame.frame;

        image_u8_t im{
            .width = frame_data.cols,
            .height = frame_data.rows,
            .stride = (int)frame_data.step, 
            .buf = frame_data.data
        };

        zarray_t *detections = apriltag_detector_detect(td_, im);

        std::array<VS::points, Constants::num_ref_frames> points = get_points(detections);

        VS::CameraPoseSet current_pose;
        current_pose.camera_id = cam_id;

        for (int i = 0; i < Constants::num_ref_frames; i++) {
            VS::CameraPose pose_in_current_set;

            VS::CameraPose pose_and_uncertainty = estimate_pose(points[i], id);
            pose_in_current_set.apriltag_set_number = i;

            current_pose.camera_poses.push_back(pose_in_current_set);
        }

        output_pose_queue.push(current_pose);
    };
}