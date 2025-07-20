cmake_minimum_required(VERSION 3.0.2)
project(perception)

###############################
## Extra compilation options ##
###############################

# Set the new policy for IN_LIST
cmake_policy(SET CMP0057 NEW)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
endif()

option(ENABLE_DOC_GENERATION "Enable documentation generation" OFF)

#####################################
## Find colcon macros and libraries
#####################################

find_package(ament_cmake REQUIRED)
find_package(apriltag REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(library REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# # Generate everything.
# rosidl_generate_interfaces(${PROJECT_NAME}
#  DEPENDENCIES std_msgs
# )

###########
## Build ##
###########

# Let the compiler search for headers in the include folder
include_directories(include ${apriltag_ros_INCLUDE_DIRS})

# Define the node target.
add_executable(apriltag_segmentation src/apriltag_segmentation.cpp src/image_segmentation.cpp)
# Link the relevant libraries.
target_link_libraries(apriltag_segmentation apriltag)
# Link the relevant ament packages.
ament_target_dependencies(apriltag_segmentation 
  apriltag
  cv_bridge 
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  library
  interfaces
  std_msgs
  tf2
)

#############
## Install ##
#############

#-----------------------------------------------------------------------------
# Install the cpp nodes.
#-----------------------------------------------------------------------------

install(
  TARGETS
    apriltag_segmentation
  DESTINATION lib/${PROJECT_NAME}
)

#-----------------------------------------------------------------------------
# Install the python nodes.
#-----------------------------------------------------------------------------

install(
  PROGRAMS
    scripts/robot_segmentation.py
  DESTINATION lib/${PROJECT_NAME}
)

#-----------------------------------------------------------------------------
# Install launch files
#-----------------------------------------------------------------------------

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*launch*"
)

#-----------------------------------------------------------------------------
# Install configuration files (YAML)
#-----------------------------------------------------------------------------

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

#-----------------------------------------------------------------------------
# Install additional files (if needed)
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Install package manifest
#-----------------------------------------------------------------------------

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###################
## Documentation ##
###################

## If we set the documentation generation flag
if(ENABLE_DOC_GENERATION)
    ## Look if we have the documentation generation package installed
    find_package(rosdoc_lite REQUIRED)
    ## (Re)generate package documentation with rosdoc_lite
    execute_process(COMMAND rospack find ${PROJECT_NAME}
    OUTPUT_VARIABLE PACKAGE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND roscd ${PACKAGE_DIR} && rm -r doc/html)
    execute_process(COMMAND rosdoc_lite ${PACKAGE_DIR} -o ${PACKAGE_DIR}/doc -q)
endif()


###################
## Finalization ###
###################

# Register the package.
ament_package()
