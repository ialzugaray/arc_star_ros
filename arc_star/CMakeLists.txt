cmake_minimum_required(VERSION 2.8.8)
project(arc_star)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(arc_star_app_file src/arc_star_app_file.cc src/arc_star_detector.cc)
target_link_libraries(arc_star_app_file)

