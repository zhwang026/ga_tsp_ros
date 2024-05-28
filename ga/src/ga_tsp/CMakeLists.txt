cmake_minimum_required(VERSION 2.8.3)
project(ga_tsp)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 #LIBRARIES ga_tsp
 CATKIN_DEPENDS roscpp std_msgs visualization_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_executable(ga_tsp_node
    src/gatsp_node.cpp 
    src/ga.cpp 
    )
target_link_libraries(ga_tsp_node
    ${catkin_LIBRARIES} 
    )  
