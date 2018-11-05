cmake_minimum_required(VERSION 2.8.8)
project(AsynchronousCornerDetector)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${eigen3_INCLUDE_DIRS}
)

add_executable(arc_star_app_file src/arc_star_app_file.cc src/arc_star_detector.cc)
target_link_libraries(arc_star_app_file)



FILE(GLOB_RECURSE CcFiles "src/*.cc")
FILE(GLOB_RECURSE HppFiles "include/*.hpp")
FILE(GLOB_RECURSE HFiles "include/*.h")
add_custom_target(sources_cc SOURCES ${CcFiles})
add_custom_target(headers_hpp SOURCES ${HppFiles})
add_custom_target(headers_h SOURCES ${HFiles})



