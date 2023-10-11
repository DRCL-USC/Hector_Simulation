cmake_minimum_required(VERSION 3.5)
project(hector_control)
set(CMAKE_CXX_STANDARD 14)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs  
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
#find_package(qpoases)
catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    #${GAZEBO_LIBRARY_DIRS}
)

link_directories(
    include/sdk/lib
    ${GAZEBO_LIBRARY_DIRS}
)

add_subdirectory(third_party/qpOASES)

file(GLOB_RECURSE SRC_LIST
"src/*/*.cpp"
"src/*/*.h"
"ConvexMPC/*.cpp"
)
add_executable(hector_ctrl src/main.cpp  ${SRC_LIST} )
target_link_libraries(hector_ctrl ${catkin_LIBRARIES} qpOASES -pthread lcm)


