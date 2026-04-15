# Minimum CMake version requirement and project name definition
cmake_minimum_required(VERSION 3.8)
project(aruco_detector_skill_server)

#############################################################
# COMPILER SETTINGS
#############################################################

# Set C++17 standard if not already set
# Important for using modern C++ features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable compiler warnings to improve code quality
# -Wall: all warnings, -Wextra: extra warnings, -Wpedantic: strict standard compliance
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#############################################################
# DEPENDENCIES
#############################################################

# Find required ROS 2 packages
# ament_cmake: ROS 2 build system
# rclcpp: ROS 2 C++ client library
# rclcpp_action: ROS 2 action support
# aruco_detector_skill_msgs: custom messages for this skill
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(aruco_detector_skill_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)



# Store dependencies in a variable for later use
# This simplifies adding/removing dependencies in a single place
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  
  aruco_detector_skill_msgs
  ament_index_cpp
)

#############################################################
# COMMON LIBRARY (OTHER LIBRARIES DEFINITION)
#############################################################

# Header-only utility library (commons)
# A header-only library consists only of .hpp files (no source .cpp files)
# For header-only libraries, we don't need to create compile targets
# We simply need to ensure the header files are:
# 1. Available during compilation via include paths
# 2. Properly installed to the include directory
#
# This approach is used for:
# - Template-based libraries
# - Small utility functions defined entirely in headers
# - Libraries with inline functions for performance

# Note: If you later add .cpp files to this library, you would need to
# uncomment the following and create a proper library target:
#
# add_library(mycommons
#   src/common/verbosity_levels.cpp
#   src/common/std_vector_operations.cpp
# )
#
# target_include_directories(mycommons PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
#
# ament_target_dependencies(mycommons
#   rclcpp
# )

#############################################################
# MAIN LIBRARY
#############################################################

# Create shared library from source files
# The name is defined by the PROJECT_NAME variable (aruco_detector_skill_server)
add_library(${PROJECT_NAME}
        src/aruco_detector_skill_server.cpp
        # Commented files to be added later
        # src/heuristics/protein_selector.cpp
        # src/heuristics/fiber_selector.cpp
)

# Define include directories for the library
# BUILD_INTERFACE: directories used during build
# INSTALL_INTERFACE: directories used after installation
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add ROS 2 dependencies to the library
# The ament_target_dependencies command automatically sets up:
# - Include directories from dependencies
# - Required compiler flags
# - Links to dependency libraries
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

#############################################################
# EXECUTABLE
#############################################################

# Create the ROS 2 node executable from the main source file
add_executable(${PROJECT_NAME}_node src/aruco_detector_skill_server_node.cpp)

# IMPORTANT: Link the executable to the main library
# target_link_libraries is the correct command for linking to local libraries
# This ensures the executable has access to all functions implemented in the library
# Dependencies from the library are also automatically propagated to the executable (i.e the ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############################################################
# INSTALLATION
#############################################################

# Configure installation of the library and executable
# TARGETS: specifies which targets will be installed
# EXPORT: creates an export file for use by other packages
# ARCHIVE/LIBRARY/RUNTIME: destinations for different types of files
install(
        TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files maintaining directory structure
install(DIRECTORY include/ DESTINATION include)

# Install the export file
# This allows other packages to find and use this package via find_package
# NAMESPACE: adds a prefix to exported target names
install(
        EXPORT export_${PROJECT_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION share/${PROJECT_NAME}/cmake
)

# Install launch files to make them available for ros2 launch command
install(DIRECTORY
        launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

# Optional: also install configuration files if needed
install(DIRECTORY
        config/
        DESTINATION share/${PROJECT_NAME}/config
)

#############################################################
# EXPORT
#############################################################

# Export targets for integration with the ament system
# This allows other packages to use find_package() to find this package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies so packages depending on this one also know
# which other dependencies are needed (dependency propagation)
ament_export_dependencies(${DEPENDENCIES})

#############################################################
# IDE SUPPORT
#############################################################

# Find all header files to help IDEs like CLion
# This doesn't affect compilation, it just improves developer experience
file(GLOB_RECURSE HEADER_FILES include/*.hpp include/*.h)
add_custom_target(${PROJECT_NAME}_headers SOURCES ${HEADER_FILES})

#############################################################
# FINALIZATION
#############################################################

# Finalize ament package configuration
# This command should be the last one in CMakeLists.txt
ament_package()
