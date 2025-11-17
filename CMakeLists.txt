cmake_minimum_required(VERSION 3.10)
project(fake_object_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tuw_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)

# Plugin-Library
add_library(${PROJECT_NAME} SHARED
  src/fake_object_filter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs pluginlib filters
  tf2 tf2_ros tf2_geometry_msgs
  tuw_geometry
  geometry_msgs
)

# pluginlib: plugin.xml im ament-index registrieren
pluginlib_export_plugin_description_file(filters plugin.xml)

# Node (Executable)
add_executable(object_filter_node
  src/object_filter_node.cpp
)
add_executable(rectangle_node 
  src/rectangle_node.cpp
  )

target_include_directories(object_filter_node PRIVATE include)

ament_target_dependencies(object_filter_node
  rclcpp sensor_msgs pluginlib filters
  tf2 tf2_ros tf2_geometry_msgs
  tuw_geometry
  )
ament_target_dependencies(rectangle_node
  rclcpp
  geometry_msgs
)

target_link_libraries(object_filter_node ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME} 
  object_filter_node
  rectangle_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

# Falls vorhanden
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()

ament_package()