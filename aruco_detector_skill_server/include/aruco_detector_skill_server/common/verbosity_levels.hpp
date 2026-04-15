/**
 * \file verbosity_levels.hpp
 * \brief Utility functions for handling verbosity levels in ROS2
 *
 * @version 1.0
 * @author Author Name
 */

#ifndef ARUCO_DETECTOR_SKILL_SERVER_COMMON_VERBOSITY_LEVELS_HPP
#define ARUCO_DETECTOR_SKILL_SERVER_COMMON_VERBOSITY_LEVELS_HPP

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace aruco_detector_skill {
namespace verbosity_levels {

    /**
     * @brief Get the current logger level of a node
     *
     * @param _node Pointer to the ROS2 node
     * @return std::string String representation of the current logger level
     * @throws std::runtime_error If an invalid logger level is retrieved
     */
    inline std::string getNodeLoggerLevel(rclcpp::Node* _node) {
        auto level = _node->get_logger().get_effective_level();

        switch (level) {
        case rclcpp::Logger::Level::Debug:
            return "DEBUG";
        case rclcpp::Logger::Level::Info:
            return "INFO";
        case rclcpp::Logger::Level::Warn:
            return "WARN";
        case rclcpp::Logger::Level::Error:
            return "ERROR";
        case rclcpp::Logger::Level::Fatal:
            return "FATAL";
        default:
            throw std::runtime_error("[" + std::string(_node->get_name()) +"] Invalid retrieved logger level. ");
        }
    }

    /**
     * @brief Set the logger level of a node
     *
     * @param _node Pointer to the ROS2 node
     * @param _level String representation of log level ("DEBUG", "INFO", "WARN", "ERROR", "FATAL")
     * @throws std::runtime_error If an invalid logger level string is provided
     */
    inline void setNodeLoggerLevel(rclcpp::Node* _node, const std::string& _level) {
        rclcpp::Logger::Level level_enum;

        if (_level == "DEBUG") {
            level_enum = rclcpp::Logger::Level::Debug;
        } else if (_level == "INFO") {
            level_enum = rclcpp::Logger::Level::Info;
        } else if (_level == "WARN") {
            level_enum = rclcpp::Logger::Level::Warn;
        } else if (_level == "ERROR") {
            level_enum = rclcpp::Logger::Level::Error;
        } else if (_level == "FATAL") {
            level_enum = rclcpp::Logger::Level::Fatal;
        } else {
            throw std::runtime_error("[" + std::string(_node->get_name()) +"] Invalid logger level: " + _level);
        }

        _node->get_logger().set_level(level_enum);
        RCLCPP_INFO_STREAM(_node->get_logger(), "Logger level changed to: " << verbosity_levels::getNodeLoggerLevel(_node));
    }

} /* namespace verbosity_levels */
} /* namespace aruco_detector_skill */

#endif /* ARUCO_DETECTOR_SKILL_SERVER_COMMON_VERBOSITY_LEVELS_HPP */