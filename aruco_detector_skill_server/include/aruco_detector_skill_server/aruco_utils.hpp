#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#ifndef ARUCO_DETECTOR_SKILL_SERVER_ARUCO_UTILS_H
#define ARUCO_DETECTOR_SKILL_SERVER_ARUCO_UTILS_H

namespace aruco_detector_skill::utils {

class ArucoUtils {
protected:
  cv::aruco::Dictionary dict_;
  cv::aruco::DetectorParameters params_;

  cv::aruco::ArucoDetector detector_;
  void detect(const cv::Mat &image,
              std::vector<std::vector<cv::Point2f>> &corners,
              std::vector<std::vector<cv::Point2f>> &rejected_corners,
              std::vector<int> &ids);

public:
  explicit ArucoUtils(
      cv::aruco::PredefinedDictionaryType dict_type = cv::aruco::DICT_4X4_50,
      const cv::aruco::DetectorParameters &params =
          cv::aruco::DetectorParameters())
      : dict_(cv::aruco::getPredefinedDictionary(dict_type)), params_(params),
        detector_(dict_, params_) {}
  ~ArucoUtils() = default;
};
} // namespace aruco_detector_skill::utils
#endif // ARUCO_DETECTOR_SKILL_SERVER_ARUCO_UTILS_H
