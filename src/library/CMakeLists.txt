cmake_minimum_required(VERSION 3.0.2)
project(library)

###############################
## Extra compilation options ##
###############################

# Set the new policy for IN_LIST
cmake_policy(SET CMP0057 NEW)

# # Compile with debug symbols if the user sets -DCMAKE_BUILD_TYPE=Debug when calling catkin
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  # Add debug symbols
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0") # Disable optimization
# endif()

## Add a custom compile option to enable or not the documentation generation
# option(ENABLE_DOC_GENERATION "Enable documentation generation" OFF)

#####################################
## Find colcon macros and libraries
#####################################

find_package(ament_cmake REQUIRED COMPONENTS)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(interfaces REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
# )

################################################
## Load in-package dependencies ################
################################################

###########
## Build ##
###########

# Let the compiler search for headers in the include folder
include_directories(include)

# Define the library target.
add_library(library
  src/ros2_node.cpp
)

ament_target_dependencies(library
  rclcpp rcl_interfaces interfaces)
ament_export_targets(library HAS_LIBRARY_TARGET)

# add_executable(my_node src/my_node.cpp)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#############
## Install ##
#############

# Install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/library
  DESTINATION include
)

# Install libraries
install(
  TARGETS library
  EXPORT library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## Install nodes.
# install(TARGETS my_node
#   DESTINATION lib/${PROJECT_NAME})

# # Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

###################
## Documentation ##
###################

# ## If we set the documentation generation flag
# if(ENABLE_DOC_GENERATION)
#     ## Look if we have the documentation generation package installed
#     find_package(rosdoc_lite REQUIRED)
#     ## (Re)generate package documentation with rosdoc_lite
#     execute_process(COMMAND rospack find ${PROJECT_NAME}
#     OUTPUT_VARIABLE PACKAGE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
#     execute_process(COMMAND roscd ${PACKAGE_DIR} && rm -r doc/html)
#     execute_process(COMMAND rosdoc_lite ${PACKAGE_DIR} -o ${PACKAGE_DIR}/doc -q)
# endif()


###################
## Finalization ###
###################

# Register the package.
ament_package()
