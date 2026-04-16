
/**
 * \file aruco_detector_server.cpp
 * \brief ArucoDetectorSkillServer class definition
 *
 * @version 1.0
 * @author Author Name
 */

#include "aruco_detector_skill_server/aruco_detector_skill_server.hpp"
#include <opencv2/objdetect/aruco_dictionary.hpp>

ArucoDetectorSkillServer::ArucoDetectorSkillServer(
    const rclcpp::Node::SharedPtr &_node) {
  node_ = _node;
  package_path_ = ament_index_cpp::get_package_share_directory(
      "aruco_detector_skill_server"); // package name that could be different
                                      // from node name.

  RCLCPP_INFO(node_->get_logger(), "ArucoDetectorSkillServer initialized.");
  SetupSkillConfigurationFromParameterServer();
}

void ArucoDetectorSkillServer::Start() {
  action_server_ = rclcpp_action::create_server<ArucoDetectorSkill>(
      node_, "ArucoDetectorSkill",
      std::bind(&ArucoDetectorSkillServer::handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArucoDetectorSkillServer::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&ArucoDetectorSkillServer::handle_accepted, this,
                std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Action server started.");
}

rclcpp_action::GoalResponse ArucoDetectorSkillServer::handle_goal(
    const rclcpp_action::GoalUUID &_uuid,
    std::shared_ptr<const ArucoDetectorSkill::Goal> _goal) {
  RCLCPP_INFO(node_->get_logger(), "Received goal request.");
  (void)_uuid;
  (void)_goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArucoDetectorSkillServer::handle_cancel(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle) {
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal.");
  (void)_goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArucoDetectorSkillServer::handle_accepted(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle) {
  using namespace std::placeholders; // NOLINT [build/c++11]
  std::thread{std::bind(&ArucoDetectorSkillServer::execute, this, _1),
              _goal_handle}
      .detach();
}

void ArucoDetectorSkillServer::feedback(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle,
    int _percentage, std::string _status) {
  auto feedback = std::make_shared<ArucoDetectorSkill::Feedback>();
  auto &feedback_percentage = feedback->percentage;
  auto &feedback_skill_status = feedback->skill_status;

  feedback_percentage = _percentage;
  feedback_skill_status = _status;

  _goal_handle->publish_feedback(feedback);
  RCLCPP_INFO(node_->get_logger(), "Publish feedback.");
}

void ArucoDetectorSkillServer::set_succeeded(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle,
    std::string _status, std::string _outcome) {
  auto result = std::make_shared<ArucoDetectorSkill::Result>();
  result->percentage = 100;
  if (_status.empty()) {
    result->skill_status = "ArucoDetector Skill: Succeeded";
  } else {
    result->skill_status = _status;
  }
  result->outcome = _outcome;

  _goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "Skill executed with success.");
}

void ArucoDetectorSkillServer::set_aborted(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle,
    std::string _status, std::string _outcome) {
  auto result = std::make_shared<ArucoDetectorSkill::Result>();
  result->percentage = 0;
  if (_status.empty()) {
    result->skill_status = "ArucoDetector Skill: Aborted";
  } else {
    result->skill_status = _status;
  }
  result->outcome = _outcome;

  _goal_handle->abort(result);
  RCLCPP_INFO(node_->get_logger(), "Goal Canceled");
}

void ArucoDetectorSkillServer::is_cancelled(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle) {

  if (_goal_handle->is_canceling()) {
    auto result = std::make_shared<ArucoDetectorSkill::Result>();
    result->percentage = 0;
    result->skill_status = "Goal canceled by client request";
    result->outcome = "CANCELED";

    _goal_handle->canceled(result);
    RCLCPP_INFO(node_->get_logger(), "%s", result->skill_status.c_str());
  }
}

bool ArucoDetectorSkillServer::check_preemption(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle) {
  return _goal_handle->is_canceling();
}

void ArucoDetectorSkillServer::setupLogsDirectory() {
  if (logs_path_.empty()) {
    std::string home = std::getenv("HOME");
    if (home.empty()) {
      home = "/tmp"; // Fallback if HOME is not defined
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t),
                        "%Y-%m-%d_%H-%M-%S"); // ISO-like (safe for filesystem)
    node_timestamp_id_ = ss.str();

    logs_path_ =
        home + "/.ros2/" + node_->get_name() + "/" + node_timestamp_id_;
  }
  try {
    if (!std::filesystem::exists(logs_path_)) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Logs directory not found. Creating: %s",
                   logs_path_.c_str());
      std::filesystem::create_directories(logs_path_);
      RCLCPP_DEBUG(node_->get_logger(),
                   "Logs directory created successfully: %s",
                   logs_path_.c_str());
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Logs directory identified: %s",
                   logs_path_.c_str());
    }
  } catch (const std::filesystem::filesystem_error &e) {
    std::string error_msg =
        "Failed to create logs directory at '" + logs_path_ + "': " + e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  } catch (const std::exception &e) {
    std::string error_msg =
        "Unexpected error while setting up logs directory: " +
        std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }
}

void ArucoDetectorSkillServer::SetupSkillConfigurationFromParameterServer() {

  node_->get_parameter_or("ros_verbosity_level", ros_verbosity_level_,
                          std::string("DEBUG"));
  node_->get_parameter_or("log_folder_path", logs_path_, std::string(""));

  aruco_detector_skill::verbosity_levels::setNodeLoggerLevel(
      node_.get(), ros_verbosity_level_);

  setupLogsDirectory();

  opencv_encodings_ = {{"bgr8", sensor_msgs::image_encodings::BGR8},
                       {"rgb8", sensor_msgs::image_encodings::RGB8},
                       {"mono8", sensor_msgs::image_encodings::MONO8},
                       {"mono16", sensor_msgs::image_encodings::MONO16},
                       {"32FC1", sensor_msgs::image_encodings::TYPE_32FC1},
                       {"32FC3", sensor_msgs::image_encodings::TYPE_32FC3},
                       {"16UC1", sensor_msgs::image_encodings::TYPE_16UC1},
                       {"8UC1", sensor_msgs::image_encodings::TYPE_8UC1},
                       {"8UC3", sensor_msgs::image_encodings::TYPE_8UC3},
                       {"32SC1", sensor_msgs::image_encodings::TYPE_32SC1},
                       {"32SC3", sensor_msgs::image_encodings::TYPE_32SC3}};
}

void ArucoDetectorSkillServer::ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, opencv_encodings_[msg->encoding]);

    // Do all heavy processing BEFORE acquiring the lock
    cv::Mat converted;
    const int channels = cv_ptr->image.channels();
    if (channels == 1)
      cv::cvtColor(cv_ptr->image, converted, cv::COLOR_GRAY2BGR);
    else if (channels == 3 || channels == 4)
      converted = cv_ptr->image;
    else {
      RCLCPP_ERROR(node_->get_logger(), "Unsupported number of channels: %d",
                   channels);
      return;
    }
    converted.convertTo(converted, CV_8U);

    // Critical section — as short as possible
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_ = std::move(converted);
    }
    has_image_.store(true);

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void ArucoDetectorSkillServer::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {

  const bool valid_camera_info = std::any_of(msg->k.begin(), msg->k.end(),
                                             [](double v) { return v != 0.0; });

  if (!valid_camera_info) {
    RCLCPP_WARN(node_->get_logger(),
                "Received invalid camera intrinsics (K all zeros)");
    return;
  }

  cv::Mat camera_intrinsics_matrix = cv::Mat::zeros(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_intrinsics_matrix.at<double>(i, j) = msg->k[i * 3 + j];

  const int d_size = static_cast<int>(msg->d.size());
  cv::Mat camera_distortion_coefficients_matrix =
      cv::Mat::zeros(1, d_size, CV_64F);
  for (int i = 0; i < d_size; i++)
    camera_distortion_coefficients_matrix.at<double>(0, i) = msg->d[i];

  // Critical section — both matrices updated atomically
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    camera_intrinsics_matrix_ = std::move(camera_intrinsics_matrix);
    camera_distortion_coefficients_matrix_ =
        std::move(camera_distortion_coefficients_matrix);
  }
  has_camera_info_.store(true);
}

void ArucoDetectorSkillServer::execute(
    const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle) {
  /*
  The execution of the skill should be coded here.
  In order to save you time, the methods check_preemption(), feedback(),
  set_succeeded() and set_aborted() should be used. The check_preemption()
  method should be called periodically. The variable "percentage" should be
  updated when there is an evolution in the execution of the skill. feedback()
  method should be called when there is an evolution in the execution of the
  skill.
  */

  RCLCPP_INFO(node_->get_logger(), "Executing skill.");
  const auto goal = _goal_handle->get_goal();

  action_outcome_ = "succeeded";

  bool action_success = false;

  switch (goal->operation) {
  case OperationMode::Detect:
    if (DetectAruco()) {
      action_success = true;
    }
    break;
  case OperationMode::Load:
    if (LoadDetector()) {
      action_success = true;
    }
    break;
  }

  (action_success) ? set_succeeded(_goal_handle, "succeeded", action_outcome_)
                   : set_aborted(_goal_handle, "aborted", "aborted");
}

cv::aruco::PredefinedDictionaryType
ArucoDetectorSkillServer::dictionaryFromString(const std::string &name) {
  static const std::unordered_map<std::string,
                                  cv::aruco::PredefinedDictionaryType>
      dict = {
          {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
          {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
          {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
          {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
          {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
          {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
          {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
          {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
          {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
          {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
          {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
          {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
          {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
      };

  auto it = dict.find(name);
  if (it == dict.end()) {
    throw std::invalid_argument("Unknown ArUco dictionary: '" + name + "'");
  }
  return it->second;
}

bool ArucoDetectorSkillServer::LoadDetector() { return true; }

bool ArucoDetectorSkillServer::DetectAruco() {
  // Only for tests
  cv::aruco::DetectorParameters detectorParams =
      cv::aruco::DetectorParameters();
  detectorParams.adaptiveThreshConstant = 1.0;

  auto dic_type = dictionaryFromString("DICT_4X4_50");

  // Snapshot method to get the image
  if (!has_image_.load()) {
    RCLCPP_WARN(node_->get_logger(), "No image received yet!");
    return false;
  }

  if (!has_camera_info_.load()) {
    RCLCPP_WARN(node_->get_logger(), "No camera info received yet!");
    return false;
  }

  cv::Mat local_image;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    local_image = latest_image_.clone();
  }

  cv::Mat local_camera_intrinsics_matrix,
      local_camera_distortion_coefficients_matrix;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    local_camera_intrinsics_matrix = camera_intrinsics_matrix_.clone();
    local_camera_distortion_coefficients_matrix =
        camera_distortion_coefficients_matrix_.clone();
  }

  // Image processing steps?

  aruco_detector_skill::utils::ArucoUtils aruco_detector(dic_type,
                                                         detectorParams);
  // aruco_detector.Detect(image_grayscale, );

  // Do something with pose

  has_image_.store(false);
  return true;
}
