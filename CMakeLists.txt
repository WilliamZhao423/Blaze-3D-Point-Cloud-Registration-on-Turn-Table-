cmake_minimum_required(VERSION 2.8.3)

project(lidar_slam_3d)

cmake_policy(SET CMP0021 OLD) ## TODO @2022.11.05

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_BUILD_TYPE "Release")
##set(CMAKE_CXX_FLAGS "-std=c++11 -o3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations -Wno-deprecated-declarations -Wno-error=pessimizing-move -Wno-terminate")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)
# find_package(G2O REQUIRED)
find_package(Ceres REQUIRED)
find_package(Cholmod REQUIRED)
find_package(PCL REQUIRED)

# set(G2O_LIBS
#   ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} g2o_types_sba g2o_types_slam3d
#)

catkin_package(
)

include_directories(
  "/usr/include/eigen3"
  src
  include/lidar_slam_3d
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  ${G2O_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
)

add_executable(lidar_slam_3d
  src/floor_filter.cpp
  src/map_builder.cpp
  src/lidar_slam_3d_ros.cpp
  src/node.cpp
)

target_link_libraries(lidar_slam_3d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
#  ${G2O_LIBS}
  ${CERES_LIBRARIES}
)

install(TARGETS lidar_slam_3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

