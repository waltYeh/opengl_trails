cmake_minimum_required(VERSION 2.8.3)
project(opengl_trails)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opengl_trails
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(mesh SHARED src/mesh.cpp)

add_executable(main src/main.cpp)
target_link_libraries(main
  mesh 
 ${catkin_LIBRARIES} 
  )
target_link_libraries(main GLU GL glut)
add_dependencies(main main_cpp)

