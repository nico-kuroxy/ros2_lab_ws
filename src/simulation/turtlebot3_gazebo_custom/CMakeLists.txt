# Minimal CMakeLists.txt file required to install the ressources of this package.
# In the original Turtlebot3 Project, turtlebot3_gazebo is built under the scope of the turtlebot3_simulation package.
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_gazebo_custom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
