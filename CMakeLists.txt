cmake_minimum_required(VERSION 2.8.3)
project(opencv_tutorial)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
    include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/opencv_test.cpp src/laneDetection.cpp src/calibration.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
)

