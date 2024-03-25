cmake_minimum_required(VERSION 3.5)
project(rviz_tool_path_display)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)

qt5_wrap_cpp(MOC_FILES src/rviz_tool_path_display.h)
add_library(${PROJECT_NAME} SHARED src/rviz_tool_path_display.cpp ${MOC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
rviz_common::rviz_common
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_target_dependencies(${PROJECT_NAME} rviz_common pluginlib)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rviz_common
  geometry_msgs
  rclcpp
  pluginlib
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY icons DESTINATION include/${PROJECT_NAME}/)

install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME}/)

ament_package()