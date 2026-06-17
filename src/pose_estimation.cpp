#include "Eigen/Dense"

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

                for (int corner = 0; corner < 4; corner ++) { // <----------------------------- Possible optimizations: Memory alocation of vector, and pose inversion to custom closed form function
                    Eigen::Vector4d obj_point_eigen = Constants::apriltag_poses_in_global[det->id - 1].inverse() * obj_point_choices[corner];
                    cv::point3d obj_point = {obj_point_eigen[0], obj_point_eigen[1], obj_point_eigen[2]};
                    
                    cv::point2f im_point = {det->p[corner][0], det->p[corner][1]}

                    out[j].obj_points.pushback(obj_point);
                    out[j].img_points.pushback(im_point);
                }
            }
        }
    }
}

