cmake_minimum_required(VERSION 2.8.3)
project(auto_decider)
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  parsian_robot_status.msg
  vector2D.msg
  parsian_world_model.msg
  parsian_robot.msg
  result.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  LIBRARIES auto_decider
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS nodelet roscpp std_msgs
)
add_library(auto_decider
  auto_decider_nodelet.cpp
)
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(auto_decider
  ${catkin_LIBRARIES}
)
if(catkin_EXPORTED_LIBRARIES)
add_dependencies(auto_decider ${catkin_EXPORTED_LIBRARIES})
endif()
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES auto_decider.xml auto_decider.launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
