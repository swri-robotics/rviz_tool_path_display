cmake_minimum_required(VERSION 3.5)
project(rviz_tool_path_display)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS})

qt5_wrap_cpp(MOC_FILES src/rviz_tool_path_display.h)

add_library(${PROJECT_NAME} SHARED src/rviz_tool_path_display.cpp ${MOC_FILES})
ament_target_dependencies(${PROJECT_NAME} geometry_msgs rclcpp rviz_common rviz_rendering)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME}/)

install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME}/)

ament_package()