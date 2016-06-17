cmake_minimum_required(VERSION 2.8.3)
project(dwl_terrain)

# Finding external libraries
find_package(catkin  REQUIRED COMPONENTS
  dwl
  dwl_msgs
  roscpp
  message_generation
  std_msgs
  octomap_msgs
  std_srvs
  tf
  tf_conversions
  pcl_ros)

find_package(dwl  REQUIRED)
find_package(octomap  REQUIRED)


# Adding the message files
add_message_files(FILES  RewardCell.msg
                         RewardMap.msg
                         Cell.msg
                         ObstacleMap.msg)

# Generating the messages
generate_messages(DEPENDENCIES  std_msgs)


catkin_package(
  INCLUDE_DIRS  include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS  roscpp octomap_msgs message_runtime
  DEPENDS  system_lib eigen)


# Include directories
include_directories(include  ${catkin_INCLUDE_DIRS}
                             ${dwl_INCLUDE_DIRS}
                             ${OCTOMAP_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS}
                 ${dwl_LIBRARY_DIRS}
                 ${OCTOMAP_LIBRARY_DIRS})


## Declare a cpp library
add_library(${PROJECT_NAME}  src/RewardMapSubscriber.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}
                                       ${dwl_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${dwl_terrain_EXPORTED_TARGETS})


## Declare a cpp executable
add_executable(reward_map_server  src/RewardMapServer.cpp)
add_dependencies(reward_map_server  ${catkin_EXPORTED_TARGETS})
target_link_libraries(reward_map_server  ${catkin_LIBRARIES}
                                         ${dwl_LIBRARIES}
                                         ${OCTOMAP_LIBRARIES})
add_dependencies(reward_map_server  ${PROJECT_NAME}_gencpp)

add_executable(obstacle_map_server  src/ObstacleMapServer.cpp)
add_dependencies(obstacle_map_server  ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_map_server  ${catkin_LIBRARIES}
                                           ${dwl_LIBRARIES}
                                           ${OCTOMAP_LIBRARIES})
add_dependencies(obstacle_map_server ${PROJECT_NAME}_gencpp)

add_executable(default_flat_terrain  src/DefaultFlatTerrain.cpp)
add_dependencies(default_flat_terrain  ${catkin_EXPORTED_TARGETS})
target_link_libraries(default_flat_terrain  ${catkin_LIBRARIES}
                                            ${dwl_LIBRARIES}
                                            ${OCTOMAP_LIBRARIES})