
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
    RCLCPP_INFO(node_->get_logger(), result->skill_status.c_str());
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

  aruco_detector_skill::utils::ArucoUtils detector(dic_type, detectorParams);
  return true;
}
