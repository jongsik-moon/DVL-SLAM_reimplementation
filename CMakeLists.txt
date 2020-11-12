cmake_minimum_required(VERSION 3.0.2)
project(dvl_slam_modify)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(catkin REQUIRED COMPONENTS
  image_transport
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV 3.4 REQUIRED)

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(DLib REQUIRED)
find_package(DBoW2 REQUIRED)
find_library(cholmod REQUIRED)
find_package(G2O REQUIRED)


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dvl_slam_modify
  src/main.cpp
  src/Config.cpp
  src/Frame.cpp
  src/Tracker.cpp
  src/GraphOptimizer.cpp
  src/Sensor.cpp
  src/System.cpp
  src/KeyFrame.cpp
)

target_link_libraries(dvl_slam_modify
  ${catkin_LIBRARIES}
  yaml-cpp
  ${EIGEN3_LIBS}
  ${Sophus_LIBS}
  ${DBoW2_LIBS}
  ${DLib_LIBS}
  ${CHOLMOD_LIBRARIES}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_EIGEN}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SBA}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
)
