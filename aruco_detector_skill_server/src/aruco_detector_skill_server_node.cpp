/**
 * \file aruco_detector_server_node.cpp
 * \brief ArucoDetectorSkill ros2 node definition
 *
 * @version 1.0
 * @author Author Name
 */

#include "aruco_detector_skill_server/aruco_detector_skill_server.hpp"

using namespace std;

// Application entry point.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("aruco_detector_skill_server");
  const auto skill_server = std::make_unique<ArucoDetectorSkillServer>(node);
  skill_server->start();

  rclcpp::spin(node);
  rclcpp::shutdown();

}
