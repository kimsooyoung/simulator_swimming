cmake_minimum_required(VERSION 2.8.3)
project(solar_sys_formation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
  message_generation
)

add_message_files(FILES 
  msgCoordinate.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs   
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES solar_sys_formation
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs message_runtime
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(moon_node src/moon_node.cpp)
add_dependencies(moon_node solar_sys_formation_generate_messages_cpp)
target_link_libraries(moon_node ${catkin_LIBRARIES})

add_executable(earth_node src/earth_node.cpp)
add_dependencies(earth_node solar_sys_formation_generate_messages_cpp)
target_link_libraries(earth_node ${catkin_LIBRARIES})

add_executable(start_moon_node src/start_moon_node.cpp)
add_dependencies(start_moon_node solar_sys_formation_generate_messages_cpp)
target_link_libraries(start_moon_node ${catkin_LIBRARIES})

add_executable(start_earth_node src/start_earth_node.cpp)
add_dependencies(start_earth_node solar_sys_formation_generate_messages_cpp)
target_link_libraries(start_earth_node ${catkin_LIBRARIES})


