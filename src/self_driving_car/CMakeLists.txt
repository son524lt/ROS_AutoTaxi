cmake_minimum_required(VERSION 3.0.2)
project(self_driving_car)

find_package(catkin REQUIRED COMPONENTS
  fake_localization
  gazebo_ros
  message_generation
  message_runtime
  pcl_ros
  prius_description
  prius_msgs
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  teb_local_planner
  tf2_ros
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS /usr/include
#  LIBRARIES self_driving_car
#  CATKIN_DEPENDS fake_localization gazebo_ros message_generation message_runtime pcl_ros prius_description prius_msgs robot_state_publisher roscpp rospy sensor_msgs std_msgs teb_local_planner tf2_ros
#  DEPENDS system_lib
)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} 
)

find_package(ignition-msgs1 REQUIRED)
find_package(gazebo 11 REQUIRED)
add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES} gazebo) 
target_include_directories(PriusHybridPlugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/lane_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
# add_executable(keyboard src/keyControl.cpp)
# target_link_libraries(keyboard ${catkin_LIBRARIES})

# add_executable(demo src/main.cpp)
# target_link_libraries(demo ${catkin_LIBRARIES})

# add_executable(lanefl src/lane_follow.py)

# set_target_properties(lanefl PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

