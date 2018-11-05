cmake_minimum_required(VERSION 2.8.8)
project(arc_star_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dvs_msgs
)


# export the dependencise of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS 
    arc_star/include
    ${catkin_INCLUDE_DIRS}
)



# Include directories
include_directories(
  include
  arc_star/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arc_star_ros src/arc_star_ros.cc arc_star/src/arc_star_detector.cc)
target_link_libraries(arc_star_ros ${catkin_LIBRARIES})


FILE(GLOB_RECURSE CcFiles "src/*.cc")
FILE(GLOB_RECURSE HppFiles "include/*.hpp")
FILE(GLOB_RECURSE HFiles "include/*.h")
add_custom_target(sources_cc SOURCES ${CcFiles})
add_custom_target(headers_hpp SOURCES ${HppFiles})
add_custom_target(headers_h SOURCES ${HFiles})
