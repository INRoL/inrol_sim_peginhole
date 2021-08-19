cmake_minimum_required(VERSION 3.5)
project(inrol_sim_peginhole_main)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  tf
)
  
add_message_files(FILES
  msg_for_joint.msg
)
  
generate_messages()

find_package(CUDA REQUIRED)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
 message(STATUS "Release")
 set(CMAKE_BUILD_TYPE Release)
endif()

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs std_srvs tf message_runtime
)

include(3rd/libinrol_sim_peginhole/CMakeLists.txt)
add_subdirectory(3rd/libopengl)

add_executable(simulation_main "")
target_sources(simulation_main PRIVATE src/simulation_main.cpp)
target_include_directories(simulation_main PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(
  simulation_main
  PRIVATE inrol_sim_peginhole
  ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARY} GLEW_1130 glfw
)
add_executable(visualization_main "")
target_sources(visualization_main PRIVATE src/visualization_main.cpp)
target_include_directories(visualization_main PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(
  visualization_main
  PRIVATE inrol_sim_peginhole
  ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARY} GLEW_1130 glfw
)