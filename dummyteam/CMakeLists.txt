cmake_minimum_required(VERSION 2.8.3)
project(dummyteam)

set(CMAKE_CXX_STANDARD 11)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    message_generation
    cv_bridge
    genmsg
    image_transport
    sensor_msgs
    xacro
    soccerref
)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs message_runtime soccerref)

# Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(dummy_controller src/controller.cpp)
add_dependencies(dummy_controller soccerref_generate_messages_cpp)
target_link_libraries(dummy_controller ${catkin_LIBRARIES})
