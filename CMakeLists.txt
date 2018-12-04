cmake_minimum_required(VERSION 2.8.8)
project(arc_star_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dvs_msgs
)

catkin_package(
  INCLUDE_DIRS 
    arc_star/include
    ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
  arc_star/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(arc_star_ros src/arc_star_ros.cc arc_star/src/arc_star_detector.cc)
target_link_libraries(arc_star_ros ${catkin_LIBRARIES})

