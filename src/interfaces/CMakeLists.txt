cmake_minimum_required(VERSION 3.0.2)
project(interfaces)

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
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate everything.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StatusApriltagSegmentation.msg"
  "msg/StatusCoreNode.msg"
  "srv/EnableNode.srv"
  DEPENDENCIES std_msgs
)

#############
## Include ##
#############

# # Include Cpp "include" directory
# include_directories(include)

###########
## Build ##
###########

# # Define the library target.
# add_library(library/ros2_node.cpp)
# ament_export_targets(library HAS_LIBRARY_TARGET)

# # Define the node target.
# add_executable(my_node src/my_node.cpp)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#############
## Install ##
#############

# # Install libraries
# install(
#   DIRECTORY include/library
#   DESTINATION include
# )
# install(
#   TARGETS library
#   EXPORT library
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

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
