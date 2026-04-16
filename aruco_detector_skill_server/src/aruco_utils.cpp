#include "aruco_detector_skill_server/aruco_utils.hpp"
#include <rclcpp/rclcpp.hpp>

namespace aruco_detector_skill::utils {
void ArucoUtils::detect(const cv::Mat &image,
                        std::vector<std::vector<cv::Point2f>> &corners,
                        std::vector<std::vector<cv::Point2f>> &rejected_corners,
                        std::vector<int> &ids) {
  detector_.detectMarkers(image, corners, ids, rejected_corners);
}

} // namespace aruco_detector_skill::utils
