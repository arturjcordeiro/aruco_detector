
/**
 * \file aruco_detector_skill_server.hpp
 * \brief ArucoDetectorSkillServer class declaration
 *
 * @version 1.0
 * @author Author Name
 */

#ifndef ARUCO_DETECTOR_SKILL_SERVER_HPP
#define ARUCO_DETECTOR_SKILL_SERVER_HPP

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aruco_detector_skill_msgs/action/aruco_detector_skill.hpp"
#include "aruco_detector_skill_server/aruco_utils.hpp"
#include "common/verbosity_levels.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/image_encodings.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace std::chrono_literals;

using ArucoDetectorSkill =
    aruco_detector_skill_msgs::action::ArucoDetectorSkill;
using GoalHandleArucoDetectorSkill =
    rclcpp_action::ServerGoalHandle<ArucoDetectorSkill>;

/**
 * @brief ArucoDetectorSkillServer class that provides a server for handling
 * diet estimation skills.
 *
 * This class defines a ROS 2 action server that listens for diet estimation
 * skill requests, executes the skill, and handles goal acceptance,
 * cancellation, and feedback.
 */
class ArucoDetectorSkillServer {
public:
  /**
   * @brief Constructor that receives the node as parameter
   *
   * @param _node Shared pointer to the ROS 2 Node
   */
  explicit ArucoDetectorSkillServer(const rclcpp::Node::SharedPtr &_node);

  /**
   * @brief Start the skill by creating and activating the action server
   */
  void Start();

private:
  enum OperationMode {
    Detect = 0,
  };

  std::mutex image_mutex_, camera_info_mutex_;
  cv::Mat latest_image_;
  std::atomic<bool> has_image_{false}, has_camera_info_{false};

  cv::Mat camera_intrinsics_matrix_, camera_distortion_coefficients_matrix_;

  cv::aruco::DetectorParameters detector_parameters_{
      cv::aruco::DetectorParameters()};

  rclcpp::Node::SharedPtr node_;
  std::string package_path_, ros_verbosity_level_, logs_path_,
      node_timestamp_id_, action_outcome_, image_sub_topic_, camera_info_topic_,
      image_results_publish_topic_, dict_id_string_;
  std::unordered_map<std::string, std::string> opencv_encodings_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool use_clahe_, use_adaptivethreshold_, show_rejected_,
      use_static_tf_broadcaster_, debug_tool_;
  float clahe_clip_limit_, adaptive_thresh_offset_from_mean_,
      adaptive_thresh_max_, marker_length_;
  int clahe_sizex_, clahe_sizey_, adaptive_thresh_method_,
      adaptive_thresh_type_, adaptive_thresh_blocksize_, pnp_method_;
  std::set<int> target_ids_{};

  std_msgs::msg::Header latest_header_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  std::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_results_ptr_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_results_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_subscriber_;

  std::unique_ptr<aruco_detector_skill::utils::ArucoUtils> aruco_detector_;

  rclcpp_action::Server<ArucoDetectorSkill>::SharedPtr action_server_;

  bool DetectAruco();
  static cv::aruco::PredefinedDictionaryType
  DictionaryFromString(const std::string &name);
  void ApplyClahe(cv::Mat &img_in);
  void ApplyAdaptiveThreshold(cv::Mat &img_in);
  void PublishRosImage(const cv::Mat &img, image_transport::Publisher &pub);
  void PublishPoses(std::vector<cv::Vec3d> &tvecs,
                    std::vector<cv::Vec3d> &rvecs, size_t n_markers);
  void FillPose(const cv::Vec3d &camera_rotation,
                const cv::Vec3d &camera_translation,
                geometry_msgs::msg::PoseStamped &pose_in_out);

  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void
  cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);

  /**
   * @brief Setup logs directory, creating it if necessary
   *
   * This method ensures that the logs directory exists by either
   * identifying an existing directory or creating a new one. If logs_path_
   * is empty, it defaults to package_path_ + "/logs".
   *
   * @throws std::runtime_error If directory creation fails due to
   * filesystem errors
   * @throws std::runtime_error If any unexpected error occurs during setup
   */
  void setupLogsDirectory();

  /**
   * @brief Setup skill configuration from parameter server
   */
  void SetupSkillConfigurationFromParameterServer();

  /**
   * @brief Handles the reception of a new goal
   *
   * @param _uuid The unique identifier of the goal
   * @param _goal The goal that was received
   * @return rclcpp_action::GoalResponse The response to the received goal
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &_uuid,
              std::shared_ptr<const ArucoDetectorSkill::Goal> _goal);

  /**
   * @brief Handles a cancel request for an existing goal
   *
   * @param _goal_handle The goal handle of the goal to be canceled
   * @return rclcpp_action::CancelResponse The response to the cancel request
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle);

  /**
   * @brief Starts the execution of a goal on a separate thread
   *
   * @param _goal_handle The goal handle of the accepted goal
   */
  void handle_accepted(
      const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle);

  /**
   * @brief Executes the skill associated with the goal
   *
   * @param _goal_handle The goal handle of the goal to be executed
   */
  void
  execute(const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle);

  /**
   * @brief Publishes feedback for an ongoing goal
   *
   * @param _goal_handle The goal handle of the goal for which feedback is
   * published
   * @param _percentage The current progress of the goal execution
   * @param _status A string representing the current status of the goal
   */
  void
  feedback(const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle,
           int _percentage, std::string _status);

  /**
   * @brief Marks the goal as succeeded and publishes the result
   *
   * @param _goal_handle The goal handle of the goal to be marked as succeeded
   * @param _status The status message for the succeeded goal (optional)
   * @param _outcome The outcome of the goal execution (default: "succeeded")
   */
  void set_succeeded(
      const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle,
      std::string _status = "", std::string _outcome = "succeeded");

  /**
   * @brief Marks the goal as aborted and publishes the result
   *
   * @param _goal_handle The goal handle of the goal to be marked as aborted
   * @param _status The status message for the aborted goal (optional)
   * @param _outcome The outcome of the goal execution (default: "aborted")
   */
  void
  set_aborted(const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle,
              std::string _status = "", std::string _outcome = "aborted");

  /**
   * @brief Checks if a goal has been canceled
   *
   * @param _goal_handle The goal handle to check for cancellation
   * @return true if the goal has been canceled, false otherwise
   */
  bool check_preemption(
      const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle);

  /**
   * @brief Checks if a goal has been canceled and confirms the cancellation
   *
   * @param _goal_handle The goal handle to check for cancellation
   */
  void is_cancelled(
      const std::shared_ptr<GoalHandleArucoDetectorSkill> _goal_handle);
};

#endif // ARUCO_DETECTOR_SKILL_SERVER_HPP
