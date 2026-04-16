#include "aruco_detector_skill_server/aruco_utils.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace aruco_detector_skill::utils {
void ArucoUtils::Detect(const cv::Mat &image_grayscale,
                        const cv::Mat &camera_intrinsics,
                        const cv::Mat &camera_distortion_coefficients,
                        const float marker_length,
                        std::vector<cv::Vec3d> &tvecs,
                        std::vector<cv::Vec3d> &rvecs, bool use_extrinsic_guess,
                        int pnp_flags, cv::InputOutputArray image_w_results,
                        bool show_rejected, size_t n_markers) {

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> rejected_corners;
  std::vector<int> ids;

  detector_.detectMarkers(image_grayscale, corners, ids, rejected_corners);

  // Estimate aruco's pose
  cv::Mat objPoints(4, 1, CV_32FC3);
  objPoints.ptr<cv::Vec3f>(0)[0] =
      cv::Vec3f(-marker_length / 2.f, marker_length / 2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[1] =
      cv::Vec3f(marker_length / 2.f, marker_length / 2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[2] =
      cv::Vec3f(marker_length / 2.f, -marker_length / 2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[3] =
      cv::Vec3f(-marker_length / 2.f, -marker_length / 2.f, 0);

  n_markers = corners.size();

  tvecs.reserve(n_markers);
  rvecs.reserve(n_markers);

  for (size_t i = 0; i < n_markers; i++)
    cv::solvePnP(objPoints, corners[i], camera_intrinsics,
                 camera_distortion_coefficients, rvecs[i], tvecs[i],
                 use_extrinsic_guess, pnp_flags);

  if (image_w_results.needed()) {
    cv::cvtColor(image_grayscale, image_w_results, cv::COLOR_GRAY2BGR);
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(image_w_results, corners, ids);
    }

    if (n_markers && !rvecs.empty() && !tvecs.empty()) {
      for (unsigned int i = 0; i < ids.size(); i++) {
        cv::drawFrameAxes(image_w_results, camera_intrinsics,
                          camera_distortion_coefficients, rvecs[i], tvecs[i],
                          marker_length * 1.5f, 2);
      }
    }

    if (show_rejected && !rejected_corners.empty()) {
      cv::aruco::drawDetectedMarkers(image_w_results, rejected_corners,
                                     cv::noArray(), cv::Scalar(100, 0, 255));
    }
  }
}

} // namespace aruco_detector_skill::utils
