cmake_minimum_required(VERSION 2.8.3)
project(motion_plan_occ)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 )
find_package(PCL)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_planning_rrt src/map/OccMap.cpp src/rrt/RRT.cpp src/rrt/MainRosRRT.cpp src/rrt/RRTstar.cpp src/rrt/InformedRRTstar.cpp src/rrt/HybirdRRTstar.cpp src/tools/Viewer.cpp src/rrt/HybirdRRTstarKdtree.cpp)

add_executable(rrt src/rrt/Main.cpp)

add_library(motion_planning_astar src/map/OccMap.cpp src/astar/Astar.cpp src/astar/MainRosAstar.cpp src/tools/Viewer.cpp)
add_executable(astar src/astar/Main.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(rrt
   ${catkin_LIBRARIES}
         motion_planning_rrt
         ${PCL_LIBRARIES}
 )


target_link_libraries(astar
        ${catkin_LIBRARIES}
        motion_planning_astar
        )
