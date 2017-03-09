cmake_minimum_required(VERSION 2.8.3)
project(slash_dash_bang_hash)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
	rospy
	message_generation
	std_msgs
  geometry_msgs
  std_srvs
  cv_bridge
  image_transport
  # OpenCV
	)

  set(CMAKE_CXX_FLAGS "-std=c++0x -fopenmp -O3")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
	State.msg
  MotorSpeeds.msg
  Pose2DStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slash_dash_bang_hash
 CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs eigen3 message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(Estimator
    include/Estimator/estimator.h
    src/Estimator/estimator.cpp
    include/Utilities/utilities.h
    src/Utilities/utilities.cpp)
target_link_libraries(Estimator ${catkin_LIBRARIES})

add_executable(AI
    src/Utilities/utilities.cpp
    src/AI/skills.cpp
	  src/AI/ai.cpp
   )
target_link_libraries(AI ${catkin_LIBRARIES})

add_executable(PathPlanner
    src/Utilities/utilities.cpp
    src/AI/skills.cpp
    include/PathPlanner/path_planner.h
    src/PathPlanner/path_planner.cpp)
target_link_libraries(PathPlanner ${catkin_LIBRARIES})

add_executable(Controller
    src/Utilities/utilities.cpp
    include/controller/controller.h
    src/controller/controller.cpp
  src/controller/PID.cpp)
target_link_libraries(Controller ${catkin_LIBRARIES})

add_executable(Vision
    src/Utilities/utilities.cpp
    src/Vision/vision.cpp
    include/Vision/vision.h)
target_link_libraries(Vision ${OpenCV_LIBS})
target_link_libraries(Vision ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(PathPlanner slash_dash_bang_hash_generate_messages_cpp soccerref_generate_messages_cpp)
add_dependencies(Controller soccerref_generate_messages_cpp slash_dash_bang_hash_generate_messages_cpp)
# add_dependencies(Utilities soccerref_generate_messages_cpp slash_dash_bang_hash_generate_messages_cpp)
add_dependencies(AI soccerref_generate_messages_cpp slash_dash_bang_hash_generate_messages_cpp)
add_dependencies(Estimator soccerref_generate_messages_cpp slash_dash_bang_hash_generate_messages_cpp)
