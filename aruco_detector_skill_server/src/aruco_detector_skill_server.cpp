
/**
 * \file aruco_detector_server.cpp
 * \brief ArucoDetectorSkillServer class definition
 *
 * @version 1.0
 * @author Author Name
 */

#include "aruco_detector_skill_server/aruco_detector_skill_server.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/imgcodecs.hpp>

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

  node_->get_parameter_or("RosVerbosityLevel", ros_verbosity_level_,
                          std::string("DEBUG"));
  node_->get_parameter_or("LogFolderPath", logs_path_, std::string(""));
  node_->get_parameter_or("DebugTools", debug_tool_, true);

  // Node rate and limits
  node_->get_parameter_or("RefreshRate", refresh_rate_, 10);
  node_->get_parameter_or("Timeout", timeout_seconds_, 10);

  // -- Topics to use when running "Image" action == 2
  node_->get_parameter_or("ImagePath", image_path_, std::string(""));
  node_->get_parameter_or("CameraYaml", intrinsic_path_, std::string(""));

  // ── camera topics
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("Camera.imageSubTopic", image_sub_topic_,
                          std::string(""));
  node_->get_parameter_or("Camera.cameraInfoTopic", camera_info_topic_,
                          std::string(""));

  // ── Publiser
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("Result.topic", image_results_publish_topic_,
                          std::string("result"));
  node_->get_parameter_or("Result.showRejected", show_rejected_, false);

  node_->get_parameter_or("Aruco.markerLength", marker_length_, float(0.001f));

  node_->get_parameter_or("Aruco.useMarkerIds", use_marker_ids_, false);
  if (use_marker_ids_) {
    std::vector<int64_t> ids_vector =
        node_->get_parameter_or("Aruco.markerIds", std::vector<int64_t>());
    target_ids_ = std::set<int>(ids_vector.begin(), ids_vector.end());
  }

  // ── PnP method
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("PnPMethod", pnp_method_, 0);

  node_->get_parameter_or("UseStaticTfBroadcaster", use_static_tf_broadcaster_,
                          false);

  // ── clahe parameters
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("Clahe.use", use_clahe_, true);
  node_->get_parameter_or("Clahe.clipLimit", clahe_clip_limit_, float(4.0));
  node_->get_parameter_or("Clahe.sizeX", clahe_sizex_, 2);
  node_->get_parameter_or("Clahe.sizeY", clahe_sizey_, 2);

  // ── Adaptive threshold parameters
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("AdaptiveThreshold.use", use_adaptivethreshold_,
                          false);
  node_->get_parameter_or("AdaptiveThreshold.maxValue", adaptive_thresh_max_,
                          float(255.0));
  node_->get_parameter_or("AdaptiveThreshold.method", adaptive_thresh_method_,
                          (int)cv::ADAPTIVE_THRESH_GAUSSIAN_C);
  node_->get_parameter_or("AdaptiveThreshold.type", adaptive_thresh_type_,
                          (int)cv::THRESH_BINARY);
  node_->get_parameter_or("AdaptiveThreshold.blockSize",
                          adaptive_thresh_blocksize_, 45);
  node_->get_parameter_or("AdaptiveThreshold.constantOffsetFromMean",
                          adaptive_thresh_offset_from_mean_, float(0.0));

  // ── Dictionary
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("Dictionary", dict_id_string_, std::string(""));

  // ── adaptive thresholding
  // ─────────────────────────────────────────────────────
  node_->get_parameter_or("Aruco.adaptiveThreshWinSizeMin",
                          detector_parameters_.adaptiveThreshWinSizeMin, 3);
  node_->get_parameter_or("Aruco.adaptiveThreshWinSizeMax",
                          detector_parameters_.adaptiveThreshWinSizeMax, 23);
  node_->get_parameter_or("Aruco.adaptiveThreshWinSizeStep",
                          detector_parameters_.adaptiveThreshWinSizeStep, 10);
  node_->get_parameter_or("Aruco.adaptiveThreshConstant",
                          detector_parameters_.adaptiveThreshConstant, 7.0);

  // ── Contour Filtering
  // ─────────────────────────────────────────────────────────
  node_->get_parameter_or("Aruco.minMarkerPerimeterRate",
                          detector_parameters_.minMarkerPerimeterRate, 0.03);
  node_->get_parameter_or("Aruco.maxMarkerPerimeterRate",
                          detector_parameters_.maxMarkerPerimeterRate, 4.0);
  node_->get_parameter_or("Aruco.polygonalApproxAccuracyRate",
                          detector_parameters_.polygonalApproxAccuracyRate,
                          0.03);
  node_->get_parameter_or("Aruco.minCornerDistanceRate",
                          detector_parameters_.minCornerDistanceRate, 0.05);
  node_->get_parameter_or("Aruco.minDistanceToBorder",
                          detector_parameters_.minDistanceToBorder, 3);
  node_->get_parameter_or("Aruco.minMarkerDistanceRate",
                          detector_parameters_.minMarkerDistanceRate, 0.05);

  // ── Corner Refinement
  // ──────────────────── 0=NONE, 1=SUBPIX, 2=CONTOUR, 3=APRILTAG
  node_->get_parameter_or("Aruco.cornerRefinementMethod",
                          detector_parameters_.cornerRefinementMethod, 0);
  node_->get_parameter_or("Aruco.cornerRefinementWinSize",
                          detector_parameters_.cornerRefinementWinSize, 5);
  node_->get_parameter_or("Aruco.cornerRefinementMaxIterations",
                          detector_parameters_.cornerRefinementMaxIterations,
                          30);
  node_->get_parameter_or("Aruco.cornerRefinementMinAccuracy",
                          detector_parameters_.cornerRefinementMinAccuracy,
                          0.1);
  node_->get_parameter_or(
      "Aruco.relativeCornerRefinmentWinSize", // note: OpenCV typo
      detector_parameters_.relativeCornerRefinmentWinSize, 0.3f);

  // ── Marker Decoding
  // ───────────────────────────────────────────────────────────
  node_->get_parameter_or("Aruco.markerBorderBits",
                          detector_parameters_.markerBorderBits, 1);
  node_->get_parameter_or("Aruco.perspectiveRemovePixelPerCell",
                          detector_parameters_.perspectiveRemovePixelPerCell,
                          4);
  node_->get_parameter_or(
      "Aruco.perspectiveRemoveIgnoredMarginPerCell",
      detector_parameters_.perspectiveRemoveIgnoredMarginPerCell, 0.13);
  node_->get_parameter_or("Aruco.maxErroneousBitsInBorderRate",
                          detector_parameters_.maxErroneousBitsInBorderRate,
                          0.35);
  node_->get_parameter_or("Aruco.minOtsuStdDev",
                          detector_parameters_.minOtsuStdDev, 5.0);
  node_->get_parameter_or("Aruco.errorCorrectionRate",
                          detector_parameters_.errorCorrectionRate, 0.6);

  // ── TODO: April tag
  // ─────────────────────────────────────────────────────────
  node_->get_parameter_or("Aruco.aprilTagQuadDecimate",
                          detector_parameters_.aprilTagQuadDecimate, 0.0f);
  node_->get_parameter_or("Aruco.aprilTagQuadSigma",
                          detector_parameters_.aprilTagQuadSigma, 0.0f);
  node_->get_parameter_or("Aruco.aprilTagMinClusterPixels",
                          detector_parameters_.aprilTagMinClusterPixels, 5);
  node_->get_parameter_or("Aruco.aprilTagMaxNmaxima",
                          detector_parameters_.aprilTagMaxNmaxima, 10);
  node_->get_parameter_or("Aruco.aprilTagCriticalRad",
                          detector_parameters_.aprilTagCriticalRad,
                          0.1745f); // 10*PI/180
  node_->get_parameter_or("Aruco.aprilTagMaxLineFitMse",
                          detector_parameters_.aprilTagMaxLineFitMse, 10.0f);
  node_->get_parameter_or("Aruco.aprilTagMinWhiteBlackDiff",
                          detector_parameters_.aprilTagMinWhiteBlackDiff, 5);
  node_->get_parameter_or("Aruco.aprilTagDeglitch",
                          detector_parameters_.aprilTagDeglitch, 0);

  // ── Misc
  // ──────────────────────────────────────────────────────────────────────
  node_->get_parameter_or("Aruco.detectInvertedMarker",
                          detector_parameters_.detectInvertedMarker, false);
  node_->get_parameter_or("Aruco.useAruco3Detection",
                          detector_parameters_.useAruco3Detection, false);

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

  RCLCPP_INFO(node_->get_logger(), "Finishing setting up.");

  // Camera subscriber - Image and Camera info
  image_transport_ptr_ =
      std::make_shared<image_transport::ImageTransport>(node_);
  image_subscriber_ = image_transport_ptr_->subscribe(
      image_sub_topic_, 1, &ArucoDetectorSkillServer::ImageCallback, this);

  camera_info_subscriber_ =
      node_->create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_topic_, 1,
          std::bind(&ArucoDetectorSkillServer::cameraInfoCallback, this,
                    std::placeholders::_1));

  // Results publisher
  image_transport_results_ptr_ =
      std::make_shared<image_transport::ImageTransport>(node_);
  image_results_publisher_ = image_transport_results_ptr_->advertise(
      image_results_publish_topic_, 1, true);

  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "charuco_poses", 10);

  // Tf publisher
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(),
                                                 tf2::durationFromSec(30.0));
  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Detector init
  cv::aruco::PredefinedDictionaryType dic_type =
      DictionaryFromString(dict_id_string_);
  aruco_detector_ = std::make_unique<aruco_detector_skill::utils::ArucoUtils>(
      dic_type, detector_parameters_);
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
      converted = cv_ptr->image;
    else if (channels == 3)
      cv::cvtColor(cv_ptr->image, converted, cv::COLOR_BGR2GRAY);
    else if (channels == 4)
      cv::cvtColor(cv_ptr->image, converted, cv::COLOR_BGRA2GRAY);
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
      latest_header_ = msg->header;
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
    RCLCPP_ERROR(node_->get_logger(),
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
  case OperationMode::Continuous:
    if (ContinuousArucoDetection()) {
      action_success = true;
    }
    break;
  case OperationMode::Image:
    if (OfflineImage()) {
      action_success = true;
    }
    break;
  }

  (action_success) ? set_succeeded(_goal_handle, "succeeded", action_outcome_)
                   : set_aborted(_goal_handle, "aborted", "aborted");
}

// Convert string to Opencv dictionary
cv::aruco::PredefinedDictionaryType
ArucoDetectorSkillServer::DictionaryFromString(const std::string &name) {
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

bool ArucoDetectorSkillServer::ContinuousArucoDetection() {
  auto start_time = std::chrono::steady_clock::now();
  rclcpp::Rate rate(refresh_rate_); // 10hz

  // Snapshot method to get the image
  while (rclcpp::ok()) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - start_time)
            .count();

    if (elapsed >= timeout_seconds_) {
      RCLCPP_INFO(node_->get_logger(), "Timeout reached, exiting loop");
      break;
    }

    if (!has_image_.load()) {
      RCLCPP_WARN(node_->get_logger(), "No image received yet!");
      rate.sleep();
      continue;
    }

    if (!has_camera_info_.load()) {
      RCLCPP_WARN(node_->get_logger(), "No camera info received yet!");
      rate.sleep();
      continue;
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

    if (use_clahe_) {
      ApplyClahe(local_image);
    }

    if (use_adaptivethreshold_) {
      ApplyAdaptiveThreshold(local_image);
    }

    //---- Detect aruco
    // Temporary
    bool use_extrinsic_guess{false};

    cv::Mat image_w_results;
    std::vector<cv::Vec3d> tvecs, rvecs;
    size_t n_markers = 0;

    aruco_detector_->Detect(local_image, local_camera_intrinsics_matrix,
                            local_camera_distortion_coefficients_matrix,
                            marker_length_, tvecs, rvecs, use_extrinsic_guess,
                            pnp_method_, image_w_results, show_rejected_,
                            n_markers, target_ids_);

    // TODO: Refine pose with previous extrinsic guess

    // Publish Image with results and Poses
    PublishRosImage(image_w_results, image_results_publisher_);

    PublishPoses(tvecs, rvecs, n_markers);

    has_image_.store(false);

    rate.sleep();
  }
  return true;
}

bool ArucoDetectorSkillServer::OfflineImage() {

  cv::Mat local_camera_intrinsics_matrix,
      local_camera_distortion_coefficients_matrix;

  cv::Mat local_image = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);

  cv::FileStorage fs(intrinsic_path_, cv::FileStorage::READ);

  if (fs.isOpened()) {
    fs["camera_matrix"] >> local_camera_intrinsics_matrix;
    fs["distortion_coefficients"] >>
        local_camera_distortion_coefficients_matrix;
    fs.release();

    std::cout << "Successfully loaded via OpenCV." << std::endl;
  } else {
    std::cerr << "Error: Could not open the file." << std::endl;
  }

  if (use_clahe_) {
    ApplyClahe(local_image);
  }

  if (use_adaptivethreshold_) {
    ApplyAdaptiveThreshold(local_image);
  }

  //---- Detect aruco
  // Temporary
  bool use_extrinsic_guess{false};

  cv::Mat image_w_results;
  std::vector<cv::Vec3d> tvecs, rvecs;
  size_t n_markers = 0;

  aruco_detector_->Detect(local_image, local_camera_intrinsics_matrix,
                          local_camera_distortion_coefficients_matrix,
                          marker_length_, tvecs, rvecs, use_extrinsic_guess,
                          pnp_method_, image_w_results, show_rejected_,
                          n_markers, target_ids_);

  // Publish Image with results and Poses
  PublishRosImage(image_w_results, image_results_publisher_);

  has_image_.store(false);
  return true;
}

bool ArucoDetectorSkillServer::DetectAruco() {
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

  if (use_clahe_) {
    ApplyClahe(local_image);
  }

  if (use_adaptivethreshold_) {
    ApplyAdaptiveThreshold(local_image);
  }

  //---- Detect aruco
  // Temporary
  bool use_extrinsic_guess{false};

  cv::Mat image_w_results;
  std::vector<cv::Vec3d> tvecs, rvecs;
  size_t n_markers = 0;

  aruco_detector_->Detect(local_image, local_camera_intrinsics_matrix,
                          local_camera_distortion_coefficients_matrix,
                          marker_length_, tvecs, rvecs, use_extrinsic_guess,
                          pnp_method_, image_w_results, show_rejected_,
                          n_markers, target_ids_);

  // TODO: Refine pose with previous extrinsic guess

  // Publish Image with results and Poses
  PublishRosImage(image_w_results, image_results_publisher_);

  PublishPoses(tvecs, rvecs, n_markers);

  has_image_.store(false);
  return true;
}

void ArucoDetectorSkillServer::ApplyClahe(cv::Mat &img) {
  cv::Mat img_temp;
  auto clahe =
      cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_sizex_, clahe_sizey_));
  clahe->apply(img, img_temp);
  img = img_temp;
}

void ArucoDetectorSkillServer::ApplyAdaptiveThreshold(cv::Mat &img) {
  cv::Mat img_temp;
  cv::adaptiveThreshold(img, img, adaptive_thresh_max_, adaptive_thresh_method_,
                        adaptive_thresh_type_, adaptive_thresh_blocksize_,
                        adaptive_thresh_offset_from_mean_);
  img = img_temp;
}

void ArucoDetectorSkillServer::PublishPoses(std::vector<cv::Vec3d> &tvecs,
                                            std::vector<cv::Vec3d> &rvecs,
                                            size_t n_markers) {
  geometry_msgs::msg::PoseArray charuco_poses_out;
  std::cout << std::format(
                   "Publishing poses of ({}) markers| Rvecs ({}) | Tvecs({})",
                   n_markers, rvecs.size(), tvecs.size())
            << std::endl;

  // Do something with pose
  std::vector<geometry_msgs::msg::Pose> charuco_poses;
  charuco_poses.resize(n_markers);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.resize(charuco_poses.size());

  for (size_t i = 0; i < n_markers; i++) {
    FillPose(rvecs[i], tvecs[i], charuco_poses[i]);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = latest_header_;
    transform_stamped.child_frame_id = "charuco_" + std::to_string(i);

    transform_stamped.transform.translation.x = charuco_poses[i].position.x;
    transform_stamped.transform.translation.y = charuco_poses[i].position.y;
    transform_stamped.transform.translation.z = charuco_poses[i].position.z;
    transform_stamped.transform.rotation = charuco_poses[i].orientation;

    transforms.push_back(std::move(transform_stamped));
  }

  charuco_poses_out.header = latest_header_;
  charuco_poses_out.poses = std::move(charuco_poses);
  pose_publisher_->publish(charuco_poses_out);

  // Send all transforms in a single call — more efficient than one by one
  if (use_static_tf_broadcaster_)
    static_tf_broadcaster_->sendTransform(transforms);
  else
    tf_broadcaster_->sendTransform(transforms);
}

void ArucoDetectorSkillServer::PublishRosImage(
    const cv::Mat &img, image_transport::Publisher &pub) {
  if (img.empty()) {

    RCLCPP_WARN(node_->get_logger(), "Result image is empty");
    return;
  }

  sensor_msgs::msg::Image::SharedPtr msg;
  std_msgs::msg::Header header;

  std::string encoding;
  if (img.type() == CV_8UC1) {
    encoding = "mono8"; // Grayscale
  } else if (img.type() == CV_8UC3) {
    encoding = "bgr8"; // Color (Blue-Green-Red)
  } else if (img.type() == CV_8UC4) {
    encoding = "bgra8"; // Color + Alpha
  } else {
    // Fallback or error log
    RCLCPP_WARN(node_->get_logger(), "Unknown image type: %d", img.type());
    return;
  }

  try {
    msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
  } catch (cv_bridge::Exception &e) {
    return;
  }

  // 4. Publish
  pub.publish(*msg);
}

void ArucoDetectorSkillServer::FillPose(const cv::Vec3d &camera_rotation,
                                        const cv::Vec3d &camera_translation,
                                        geometry_msgs::msg::Pose &pose_in_out) {
  cv::Mat rotation_matrix;
  cv::Rodrigues(camera_rotation, rotation_matrix);
  Eigen::Matrix3d eigen_rotation_matrix;
  cv::cv2eigen(rotation_matrix, eigen_rotation_matrix);
  Eigen::Quaterniond q(eigen_rotation_matrix);
  pose_in_out.position.x = camera_translation(0);
  pose_in_out.position.y = camera_translation(1);
  pose_in_out.position.z = camera_translation(2);
  pose_in_out.orientation.x = q.x();
  pose_in_out.orientation.y = q.y();
  pose_in_out.orientation.z = q.z();
  pose_in_out.orientation.w = q.w();
}
