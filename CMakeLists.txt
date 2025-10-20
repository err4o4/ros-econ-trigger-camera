cmake_minimum_required(VERSION 3.0.2)
project(trigger_cam)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(trigger_cam_node src/trigger_cam_node.cpp)
target_link_libraries(trigger_cam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(TARGETS trigger_cam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
