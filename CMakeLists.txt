cmake_minimum_required(VERSION 2.8.3)
project(trollnode)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  image_transport
  cv_bridge
  k2_client
  roslaunch
)
find_package( OpenCV  REQUIRED )


set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
roslaunch_add_file_check(launch)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Expression.msg
   PersonArray.msg
   PersonInterest.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trollnode
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs image_transport  cv_bridge
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(trollnode
   #src/${PROJECT_NAME}/trollnode.cpp
   src/Person.cpp
   src/setExpressionFunc.cpp
 )


add_executable(publishExpression src/publishExpression.cpp)
target_link_libraries(publishExpression ${catkin_LIBRARIES})
add_dependencies(publishExpression trollnode_generate_messages_cpp)

add_executable(lookingNode src/lookingNode.cpp)
target_link_libraries(lookingNode ${catkin_LIBRARIES})
add_dependencies(lookingNode trollnode_generate_messages_cpp)

add_executable(greetingNode src/greetingNode.cpp)
target_link_libraries(greetingNode ${catkin_LIBRARIES})
add_dependencies(greetingNode trollnode_generate_messages_cpp)


add_executable(setExpression src/setExpression.cpp src/setExpressionFunc.cpp)
target_link_libraries(setExpression ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(setExpression  trollnode_generate_messages_cpp )


add_executable(estimateInterest src/estimateInterest.cpp src/Person.cpp)
target_link_libraries(estimateInterest ${catkin_LIBRARIES} )
add_dependencies(estimateInterest  trollnode_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
