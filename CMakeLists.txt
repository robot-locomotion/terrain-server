cmake_minimum_required(VERSION 2.8.3)
project(terrain_server)

# Finding external libraries
find_package(catkin  REQUIRED COMPONENTS
  dwl
  dwl_msgs
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  octomap_msgs
  std_srvs
  tf
  tf_conversions
  pcl_ros)

find_package(octomap  REQUIRED)


# Adding the message files
add_message_files(FILES  TerrainCell.msg
                         TerrainMap.msg
                         Cell.msg
                         ObstacleMap.msg)

add_service_files(FILES  TerrainData.srv)

# Generating the messages
generate_messages(DEPENDENCIES  std_msgs
                                geometry_msgs
                                dwl_msgs)


catkin_package(
  INCLUDE_DIRS  include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS  roscpp octomap_msgs message_runtime dwl)

# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

# Include directories
include_directories(include  ${catkin_INCLUDE_DIRS}
                             ${dwl_INCLUDE_DIRS}
                             ${OCTOMAP_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS}
                 ${dwl_LIBRARY_DIRS}
                 ${OCTOMAP_LIBRARY_DIRS})


## Declare a cpp library
add_library(${PROJECT_NAME}  src/TerrainMapInterface.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}
                                       ${dwl_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${terrain_server_EXPORTED_TARGETS})


## Declare a cpp executable
add_executable(terrain_map_server  src/TerrainMapServer.cpp
								   src/TerrainMapping.cpp
								   src/feature/SlopeFeature.cpp
								   src/feature/HeightDeviationFeature.cpp
								   src/feature/CurvatureFeature.cpp)
add_dependencies(terrain_map_server  ${catkin_EXPORTED_TARGETS})
target_link_libraries(terrain_map_server  ${catkin_LIBRARIES}
                                         ${dwl_LIBRARIES}
                                         ${OCTOMAP_LIBRARIES})
add_dependencies(terrain_map_server  ${PROJECT_NAME}_gencpp)

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
