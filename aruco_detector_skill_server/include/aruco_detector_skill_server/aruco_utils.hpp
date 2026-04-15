#include <opencv2/aruco.hpp>

#ifndef ARUCO_DETECTOR_SKILL_SERVER_ARUCO_UTILS_H
#define ARUCO_DETECTOR_SKILL_SERVER_ARUCO_UTILS_H

namespace aruco_detector_skill::utils {

class ArucoUtils {
protected:
  void EstimatePose();

public:
  explicit ArucoUtils() = default;
  ~ArucoUtils() = default;
};
} // namespace aruco_detector_skill::utils
#endif // ARUCO_DETECTOR_SKILL_SERVER_ARUCO_UTILS_H
