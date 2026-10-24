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

Eigen::Matrix<double, 6, 6> VS::PoseEstimator::get_cov_mat(VS::Points actual_points, cv::Mat inliers, Eigen::Matrix<double, 6, 6> T_mat, cv::Mat rvec, cv::Mat tvec) {
    using Mat6 = Eigen::Matrix<double, 6, 6>;

    const Mat6 bad = Mat6::Identity() * 1e9;
    const int N = inliers.rows;
    const int total = actual_points.obj_points.size();
    if (N < 4) return bad;

    std::vector<cv::Point3d> obj_pts(N);
    std::vector<cv::Point2d> img_pts(N);
    for (int i = 0; i < N; i++) {
        int idx = inliers.at<int>(i, 0);
        obj_pts[i] = actual_points.obj_points.at(idx);
        img_pts[i] = actual_points.img_points.at(idx);
    }

    std::vector<cv::Point2d> reprojected;
    cv::Mat jacobian;
    cv::projectPoints(obj_pts, rvec, tvec,
                      Constants::cameras[cam_id].intrinsics,
                      Constants::cameras[cam_id].distortion_constants,
                      reprojected, jacobian);

    Eigen::VectorXd r(2 * N);
    for (int i = 0; i < N; i++) {
        r(2 * i)     = reprojected[i].x - img_pts[i].x;
        r(2 * i + 1) = reprojected[i].y - img_pts[i].y;
    }

    Eigen::MatrixXd J;
    cv::cv2eigen(jacobian.colRange(0, 6), J);

    Mat6 H = (J.transpose() * J) / Constants::pixel_sigma_sq; // (JᵀJ) / σ² or the inverse of the cov matrix

    // Convert to cov mat in a numerically stable manner
    Eigen::LDLT<Mat6> ldlt(H);
    if (ldlt.info() != Eigen::Success || ldlt.rcond() < 1e-12) return bad;
    Mat6 cov = ldlt.solve(Mat6::Identity());

    // Scale based on other uncertainty measures.
    const double chi2 = r.squaredNorm() / Constants::pixel_sigma_sq / (2.0 * N - 6.0);
    cov *= std::max(1.0, chi2);                                  // inflate only
    cov /= std::max(double(N) / total, 0.25);                    // rejected points => less trust
    
    cov = VS::transformPoseCovariance(cov, T_mat*T_mat);

    return cov;
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
        pose.pose = VS::getTransform(rvec, tvec).inverse();

        // Get pose standard deviations
        pose.uncertainty = get_cov_mat(points_in_ref_frame, inliers, pose.pose, rvec, tvec);

        pose.apriltag_set_number = points_in_ref_frame.set_id;

        out.camera_poses.push_back(pose);
    }

    return out;

}

void VS::PoseEstimator::run() {
    VS::Image frame;
    //cv::Mat gray_frame;

    VS::CameraPoseSet current_pose;
    std::vector<VS::Points> points;

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Apriltag detector constants <------------------------------------- edit these?
    td->quad_decimate = Constants::quad_decimate;
    td->quad_sigma = Constants::quad_sigma;
    td->nthreads = Constants::nthreads;
    td->refine_edges = Constants::refine_edges;

    while (true) {
        frame_queue.pop(frame);

        auto current_time_sys = std::chrono::system_clock::now();
        auto current_time = current_time_sys.time_since_epoch();
        double current_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time).count();

        const cv::Mat& gray_frame = frame.frame;

        image_u8_t im{
            .width = gray_frame.cols,
            .height = gray_frame.rows,
            .stride = (int)gray_frame.step, 
            .buf = gray_frame.data
        };

        auto start = std::chrono::high_resolution_clock::now();
        zarray_t *detections = apriltag_detector_detect(td, &im); // <------------- reference image?
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        if (zarray_size(detections) > 0) {
            points = get_points(detections);
            current_pose = estimate_pose(points);
            
            current_pose.timestamp = frame.timestamp;
            
            std::cout << "Got image in: " << current_ms - current_pose.timestamp << " ms" << std::endl;
            std::cout << "Got detections in: " << duration.count() << " ms\n";

            current_time_sys = std::chrono::system_clock::now();
            current_time = current_time_sys.time_since_epoch();
            current_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time).count();
            std::cout << "Processing time: " << current_ms - current_pose.timestamp << " ms" << std::endl;
            if (!current_pose.camera_poses.empty()) {
                std::cout << current_pose.camera_poses[0].pose << std::endl;
            }

            output_pose_queue.push(current_pose);
        }

        zarray_destroy(detections);
    };

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}
