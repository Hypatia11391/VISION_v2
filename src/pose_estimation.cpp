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

std::array<VS::points, Constants::num_ref_frames> get_points(zarray_t *detections){
    std::array<Eigen::Vector4d, 4> obj_point_choices;
    obj_point_choices[0] = Eigen::Vector3d(-Constants::tag_size/2, Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[1] = Eigen::Vector3d(Constants::tag_size/2, Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[2] = Eigen::Vector3d(Constants::tag_size/2, -Constants::tag_size/2, 0.0, 1.0);
    obj_point_choices[3] = Eigen::Vector3d(-Constants::tag_size/2, -Constants::tag_size/2, 0.0, 1.0);

    std::array<VS::points, Constants::num_ref_frames> out;

    // For each detection
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // For each set of apriltags
        for (int j = 0; j < Constants::num_ref_frames; j++) {

            // Check if the detection is is in the set
            if (!std::binary_search(Constants::apriltag_ids_for_each_frame[j].begin(), Constants::apriltag_ids_for_each_frame[j].end(), det->id)) {
                out[j].exists = false;
            }
            
            else {
                out[j].exists = true;

                for (int corner = 0; corner < 4; corner ++) { // <----------------------------- Possible optimizations: Memory alocation of vector, and pose inversion to custom closed form function, or transpositiona rather than inversion.
                    Eigen::Vector4d obj_point_eigen = Constants::apriltag_poses_in_global[det->id - 1].inverse() * obj_point_choices[corner];
                    cv::point3d obj_point = {obj_point_eigen[0], obj_point_eigen[1], obj_point_eigen[2]};
                    
                    cv::point2f im_point = {det->p[corner][0], det->p[corner][1]}

                    out[j].obj_points.pushback(obj_point);
                    out[j].img_points.pushback(im_point);
                }
            }
        }
    }

    return out;
}

std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 6, 1>> estimate_pose(VS::points points, int id_) {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;

    bool successfulPnP = cv::solvePnPRansac(points.obj_points,
                                            points.img_points,
                                            Constants::cameras[id_].intrinsics,
                                            Constants::cameras[id_].distortion_constants,
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
    cv::Mat rotation_matrix;
    Eigen::Matrix4d pose_estimate;

    // Convert the rotation vector
    cv::Rodrigues(rvec, rotation_matrix);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> eigen_rot((double*)rotation_matrix.data);
    pose_estimate.block<3,3>(0,0) = eigen_rot;

    // Convert the translation vector
    pose_estimate.block<3, 1>(0, 3) = Eigen::Vector3d::Map((double*)tvec.data);

    // Get pose standard deviations
    Eigen::Matrix<double, 6, 1> sigmas = get_stds(inliers, rvec, tvec);

    std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 6, 1>> output_pose_with_uncertainty = {pose_estimate, sigmas};

    return output_pose_with_uncertainty;

}

Eigen::Matrix<double, 6, 1> get_stds(VS::points inliers, cv::Mat rvec, cv::Mat tvec){
    std::vector<cv::Point2f> reprojected_points;
    cv::Mat jacobian;
    Eigen::Matrix<double, 2*num_inliers, 1> delta_im_pts; // <---------------------------- Wrong shape? Transpose?

    Eigen::Matrix<double, num_inliers, 6> standard_divs_out;
    
    int num_inliers = inliers.img_points.size();

    cv::projectPoints(inliers.obj_points,
                      rvec,
                      tvec,
                      Constants::cameras[id_].intrinsics,
                      Constants::cameras[id_].distortion_constants,
                      reprojected_points,
                      jacobian
    );

    std::vector<cv::Point2f> delta_im_pts_cv = reprojected_points - inliers.img_points;

    for (int point = 0; point < num_inliers, point++) {
        delta_im_pts(2*point, 0) = delta_im_pts_cv[point][0] // <------------ ToDo: Check cv:poimt indexing.
        delta_im_pts(2*point + 1, 0) = delta_im_pts_cv[point][1]
    }

    Eigen::MatrixXd eigen_jacobian;
    cv::cv2eigen(jacobian, eigen_jacobian);

    // take the inverse of the jacobian, then multiply by delta im points to get delta pose.
    Eigen::Matrix<double, 6, 2*num_inliers> J_inv= VS::jacobian_psuedo_inverse(*eigen_jacobian)
    Eigen::Matrix<double, 6, 1> delta_pose = J_inv * delta_im_pts;

    return delta_pose;
}

void run() {
    VS::Image frame;
    apriltag_detector_t *td_;
    cv::Mat& frame_data;


    while (true) {
        frame_queue.pop(frame);

        cv::Mat& frame_data = frame.frame&

        image_u8_t im{ // <------------------- can I optimize this by using pointers instead or something for image data?
            .width = frame_data.cols,
            .height = frame_data.rows,
            .stride = (int)frame_data.step, 
            .buf = frame_data.data
        };

        zarray_t *detections = apriltag_detector_detect(td_, im);

        std::array<VS::points, Constants::num_ref_frames> points = get_points(detections);
    };
}